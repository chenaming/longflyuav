#!/usr/bin/env python3
"""Read-only takeover evidence. Never publishes flight commands or calls arm/mode."""
import argparse,json,time,threading
from pathlib import Path


def diagnose(events,params):
    rows=sorted(events,key=lambda x:x['t']);states=[e for e in rows if e['kind']=='state']
    takeover=None;previous=None
    for e in states:
        d=e['data']
        if previous and previous['data']['mode']=='OFFBOARD' and d['mode']!='OFFBOARD' and d['armed']:
            takeover=e;break
        previous=e
    result={'status':'no_armed_offboard_takeover_observed','parameters':params,
            'confirmed_disarm_reason':None,'motor_rotation_or_hover_verified':False}
    if takeover is None:return result
    disarm=next((e for e in states if e['t']>=takeover['t'] and not e['data']['armed']),None)
    result.update(status='takeover_observed_no_disarm',takeover=takeover,
                  disarm=disarm,mode_after_takeover=takeover['data']['mode'])
    if disarm is None:return result
    result['takeover_to_disarm_s']=disarm['t']-takeover['t']
    texts=[e for e in rows if e['kind']=='statustext' and
           takeover['t']-.5<=e['t']<=disarm['t']+2]
    result['nearby_fcu_messages']=texts
    for e in texts:
        text=e['data']['text'].lower()
        reason=None
        if 'disarmed by landing' in text:reason='px4_landing_auto_disarm'
        elif 'disarmed by auto preflight disarming' in text:reason='px4_preflight_auto_disarm'
        elif 'disarmed by ' in text:reason=e['data']['text']
        if reason:
            result.update(status='confirmed_from_fcu_message',confirmed_disarm_reason=reason,reason_evidence=e)
            return result
    landed=[e for e in rows if e['kind']=='extended' and takeover['t']<=e['t']<=disarm['t']]
    result['landed_observations_after_takeover']=landed
    result['status']='disarm_observed_reason_not_confirmed'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=180)
    args=parser.parse_args()
    if not 10<=args.seconds<=900:parser.error('seconds must be 10..900')
    import rospy
    from mavros_msgs.msg import State,ExtendedState,RCIn,StatusText
    from mavros_msgs.srv import ParamGet
    rospy.init_node('fly_takeover_readonly_recorder',anonymous=True)
    base=Path(__file__).resolve().parent/'runtime'/'takeover_records';base.mkdir(parents=True,exist_ok=True)
    prefix=base/('takeover_'+str(time.time_ns()))
    lock=threading.Lock();events=[];previous={};closed=[False]
    logfile=prefix.with_suffix('.jsonl').open('w',encoding='utf-8')
    params={}
    get=rospy.ServiceProxy('/uav1/mavros/param/get',ParamGet)
    for name in ['COM_DISARM_LAND','COM_DISARM_PRFLT','COM_RC_OVERRIDE','RC_MAP_FLTMODE','RC_MAP_ARM_SW','RC_MAP_KILL_SW']:
        try:
            p=get(param_id=name);params[name]={'success':p.success,'integer':p.value.integer,'real':p.value.real}
        except Exception as exc:params[name]={'error':str(exc)}
    def receive(kind,m):
        if kind=='state':data={'armed':m.armed,'mode':m.mode,'connected':m.connected,'system_status':m.system_status}
        elif kind=='extended':data={'landed_state':m.landed_state}
        elif kind=='rc':data={'channels':list(m.channels),'rssi':m.rssi}
        else:data={'severity':m.severity,'text':m.text}
        event={'t':time.monotonic(),'unix':time.time(),'ros_stamp':m.header.stamp.to_sec(),
               'kind':kind,'data':data,'publisher':m._connection_header.get('callerid')}
        with lock:
            if closed[0]:return
            events.append(event);logfile.write(json.dumps(event,ensure_ascii=False)+'\n');logfile.flush()
            changed=previous.get(kind)!=data;previous[kind]=data
        if kind in ('state','extended','statustext') and changed:
            print(kind+' '+json.dumps(data,ensure_ascii=False),flush=True)
    for kind,topic,typ in [('state','/uav1/mavros/state',State),('extended','/uav1/mavros/extended_state',ExtendedState),
                          ('rc','/uav1/mavros/rc/in',RCIn),('statustext','/uav1/mavros/statustext/recv',StatusText)]:
        rospy.Subscriber(topic,typ,lambda m,k=kind:receive(k,m),queue_size=200)
    print('只读记录已开始；本脚本不会解锁、切模式或发目标。请在另一终端执行拆桨台架测试。',flush=True)
    print('原始记录：'+str(prefix.with_suffix('.jsonl')),flush=True)
    deadline=time.monotonic()+args.seconds;last_disarm=None
    try:
        while not rospy.is_shutdown() and time.monotonic()<deadline:
            with lock:snapshot=list(events)
            summary=diagnose(snapshot,params)
            if summary.get('disarm'):
                last_disarm=summary['disarm']['t']
                if time.monotonic()-last_disarm>=3:break
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        with lock:closed[0]=True;logfile.close();summary=diagnose(list(events),params)
        summary.update(updated_unix=time.time(),raw_record=str(prefix.with_suffix('.jsonl')),
                       arming_service_called=False,flight_mode_service_called=False)
        path=prefix.with_suffix('.json');path.write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
        print('诊断报告：'+str(path),flush=True)


if __name__=='__main__':main()
