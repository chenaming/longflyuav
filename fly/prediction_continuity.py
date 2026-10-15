#!/usr/bin/env python3
"""Observe model freshness without arming, mode requests, or control publishing."""
import argparse
import json
import math
from pathlib import Path
import statistics
import threading
import time
import uuid


def continuity_gaps(samples, max_age=2.0):
    """Consecutive delivery gaps; the old observation expires before the new delivery."""
    gaps=[]
    for old,new in zip(samples,samples[1:]):
        gap=new['received_ros_s']-(old['observation_stamp']+max_age)
        if gap>0:
            gaps.append({'previous_request_id':old['request_id'],'next_request_id':new['request_id'],
                         'expired_at_ros_s':old['observation_stamp']+max_age,
                         'next_received_ros_s':new['received_ros_s'],'no_fresh_model_s':gap})
    return gaps


def main():
    parser=argparse.ArgumentParser(description='未解锁观察模型结果连续性，保留2秒模型有效期。')
    parser.add_argument('--seconds',type=float,default=20.)
    args=parser.parse_args()
    if not 5<=args.seconds<=60:raise SystemExit('seconds must be between 5 and 60')
    import rospy
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log
    from mavros_msgs.msg import State,ExtendedState,StatusText
    from motor_task_guard import same_task
    rospy.init_node('fly_prediction_continuity',anonymous=True)
    state=rospy.wait_for_message('/uav1/mavros/state',State,timeout=5)
    if not state.connected or state.armed:raise SystemExit('此检查要求飞控已连接且未解锁。')
    expected={'run_id':rospy.get_param('/vla_direct_bridge/run_id'),
              'instruction':rospy.get_param('/vla_direct_bridge/active_instruction')}
    lock=threading.RLock()
    predictions=[];traces=[];rejections=[];states=[];landed=[];fcu_text=[]
    observed_armed=[False]
    def on_prediction(message):
        try:
            p=json.loads(message.data)
            if not same_task(p,expected,expected['instruction'],expected['run_id']):return
            now=rospy.Time.now().to_sec()
            if not math.isfinite(p['observation_stamp']) or not math.isfinite(p['validated_stamp']):return
            p=dict(p,received_ros_s=now,received_monotonic_s=time.monotonic(),
                   observation_age_at_receive_s=now-p['observation_stamp'])
            with lock:predictions.append(p)
        except (ValueError,KeyError,TypeError):pass
    def on_log(message):
        if message.name!='/vla_direct_bridge':return
        try:
            if message.msg.startswith('COORD_TRACE '):
                t=json.loads(message.msg[len('COORD_TRACE '):])
                if t.get('run_id')==expected['run_id'] and t.get('instruction')==expected['instruction']:
                    with lock:traces.append(t)
            elif message.msg.startswith('request id='):
                with lock:rejections.append({'received_ros_s':rospy.Time.now().to_sec(),'reason':message.msg})
        except ValueError:pass
    def on_state(message):
        with lock:
            observed_armed[0]|=message.armed
            states.append({'received_ros_s':rospy.Time.now().to_sec(),'connected':message.connected,
                           'armed':message.armed,'mode':message.mode,'system_status':message.system_status})
    def on_landed(message):
        with lock:landed.append({'received_ros_s':rospy.Time.now().to_sec(),'landed_state':message.landed_state})
    def on_text(message):
        with lock:fcu_text.append({'received_ros_s':rospy.Time.now().to_sec(),'severity':message.severity,'text':message.text})
    rospy.Subscriber('/vla/validated_prediction',String,on_prediction,queue_size=100)
    rospy.Subscriber('/rosout',Log,on_log,queue_size=200)
    rospy.Subscriber('/uav1/mavros/state',State,on_state,queue_size=20)
    rospy.Subscriber('/uav1/mavros/extended_state',ExtendedState,on_landed,queue_size=20)
    rospy.Subscriber('/uav1/mavros/statustext/recv',StatusText,on_text,queue_size=20)
    print('未解锁连续性检查开始：%.0f秒，模型有效期仍为2秒；没有控制或解锁操作。'%args.seconds,flush=True)
    start=time.monotonic();failure=None
    while time.monotonic()-start<args.seconds and not rospy.is_shutdown():
        if observed_armed[0]:failure='armed observed during read-only test';break
        if (rospy.get_param('/vla_direct_bridge/run_id')!=expected['run_id']
                or rospy.get_param('/vla_direct_bridge/active_instruction')!=expected['instruction']):
            failure='task changed during observation';break
        time.sleep(.05)
    with lock:
        predictions=list(predictions);traces=list(traces);states=list(states)
        gaps=continuity_gaps(predictions)
        ended=rospy.Time.now().to_sec()
        if predictions and ended>predictions[-1]['observation_stamp']+2.:
            gaps.append({'previous_request_id':predictions[-1]['request_id'],'next_request_id':None,
                         'no_fresh_model_s':ended-(predictions[-1]['observation_stamp']+2.),'at_end':True})
        elapsed=[t['elapsed_s'] for t in traces if math.isfinite(t.get('elapsed_s',float('nan')))]
        report=dict(expected,observed_seconds=time.monotonic()-start,model_max_age_s=2.,
                    status='aborted' if failure else ('gaps_observed' if gaps else 'no_gap_observed' if predictions else 'no_validated_predictions'),
                    failure=failure,validated_prediction_count=len(predictions),
                    http_trace_count=len(traces),rejection_count=len(rejections),gap_count=len(gaps),
                    largest_gap_s=max((g['no_fresh_model_s'] for g in gaps),default=0.),
                    http_elapsed_median_s=statistics.median(elapsed) if elapsed else None,
                    http_elapsed_p95_s=sorted(elapsed)[math.ceil(.95*len(elapsed))-1] if elapsed else None,
                    http_elapsed_max_s=max(elapsed) if elapsed else None,
                    armed_observed=observed_armed[0],gaps=gaps,predictions=predictions,
                    http_traces=traces,rejections=list(rejections),states=states,
                    landed_states=list(landed),fcu_statustext=list(fcu_text),
                    scope='read-only continuity; does not establish normal disarm or future latency guarantees')
    directory=Path(__file__).resolve().parent/'runtime';directory.mkdir(exist_ok=True)
    path=directory/('prediction_continuity_'+uuid.uuid4().hex+'.json')
    encoded=json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2)
    path.write_text(encoded,encoding='utf-8')
    (directory/'prediction_continuity_latest.json').write_text(encoded,encoding='utf-8')
    print('有效模型结果=%s，HTTP坐标返回=%s，拒绝记录=%s'%(len(predictions),len(traces),len(rejections)))
    print('连续性中断=%s，最长中断=%.6f秒'%(len(gaps),report['largest_gap_s']))
    print('HTTP总耗时：中位数=%s，P95=%s，最大=%s秒'%(report['http_elapsed_median_s'],report['http_elapsed_p95_s'],report['http_elapsed_max_s']))
    print('未观察到解锁：'+str(not observed_armed[0]))
    print('结果：'+str(report['status'])+'；报告：'+str(path))
    if failure:raise SystemExit(failure)


if __name__=='__main__':main()
