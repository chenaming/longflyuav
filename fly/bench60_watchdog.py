#!/usr/bin/env python3
"""Independent finite propeller-removed bench watchdog; never arms."""
import json,os,time
from pathlib import Path
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State,RCIn
from mavros_msgs.srv import CommandBool,CommandLong,SetMode
from motor_stop import bench_disarm
from bench60_policy import watchdog_reasons,pilot_mode_selected


def main():
    rospy.init_node('fly_bench60_watchdog')
    token=rospy.get_param('~token');parent=int(rospy.get_param('~parent_pid'))
    path=Path(__file__).resolve().parent/'runtime'/'bench30_watchdog.json'
    data={};start=time.monotonic();first_armed=None;tripped=False
    report={'token':token,'armed_seen':False,'triggers':[],'stop_results':[],'status':'ready'}
    def receive(k,m):
        if k=='lease':
            try:m=json.loads(m.data)
            except ValueError:return
            if m.get('token')!=token:return
        data[k]=(m,time.monotonic())
    for k,topic,typ in [('state','/uav1/mavros/state',State),('rc','/uav1/mavros/rc/in',RCIn),
                         ('lease','/vla/bench60/lease',String)]:
        rospy.Subscriber(topic,typ,lambda m,key=k:receive(key,m),queue_size=1)
    arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool)
    force=rospy.ServiceProxy('/uav1/mavros/cmd/command',CommandLong)
    mode=rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
    ownership=rospy.ServiceProxy('/uav1/uav_control_fsm/external_setpoint_control',CommandBool)
    ready=rospy.Publisher('/vla/bench60/watchdog_ready',String,queue_size=1,latch=True)
    ready.publish(String(data=token));initial_ch6=None;last_stop=-1e9
    def persist():
        report['updated_unix']=time.time();tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2));tmp.replace(path)
    def armed_state():
        s,t=data.get('state',(None,0))
        return s.armed if s and s.connected and time.monotonic()-t<1.5 else None
    def prepare():
        r,t=data.get('rc',(None,0));s,st=data.get('state',(None,0))
        if not r or time.monotonic()-t>.75 or len(r.channels)<9 or r.channels[2]>=1100 or r.channels[8]>=1300:
            return {'mode_sent':False,'reason':'RC not fresh and low'}
        if not s or not s.connected or time.monotonic()-st>1.5:return {'mode_sent':False,'reason':'state stale'}
        return {'mode_sent':mode(base_mode=0,custom_mode='MANUAL').mode_sent,
                'reason':'fixed propeller-removed bench stop'}
    persist()
    while not rospy.is_shutdown():
        now=time.monotonic();s,st=data.get('state',(None,0));r,rt=data.get('rc',(None,0));lease,lt=data.get('lease',({},0))
        try:os.kill(parent,0);alive=True
        except ProcessLookupError:alive=False
        if r and len(r.channels)>5 and initial_ch6 is None:initial_ch6=r.channels[5]
        if s and s.armed and first_armed is None:first_armed=now;report['armed_seen']=True
        if lease.get('normal_cleanup') and not report.get('normal_cleanup_acknowledged'):
            report['normal_cleanup_acknowledged']=True;persist()
            ready.publish(String(data=token+':normal_cleanup'))
        if pilot_mode_selected(now,first_armed,s,st,lease) or lease.get('pilot_takeover'):
            # Explicit pilot/PX4 mode takeover ends this bench lease. In
            # particular, never turn a pilot's airborne mode change into disarm.
            report.update(status='pilot_takeover',final_armed=s.armed if s else None,
                          mode=s.mode if s else None,no_disarm_on_takeover=True)
            persist();return
        reasons=watchdog_reasons(now,start,first_armed,alive,lease,lt,s,st,r,rt,initial_ch6)
        if reasons and not tripped:
            tripped=True;report['triggers'].append({'t':now,'reasons':reasons,
                'rc_channels':list(r.channels) if r else None,'rc_age_s':now-rt,
                'initial_ch6':initial_ch6,'state_age_s':now-st,
                'armed':s.armed if s else None,'mode':s.mode if s else None});persist()
        if tripped and (armed_state() is not False) and now-last_stop>1:
            result=bench_disarm(arm,force,armed_state,propellers_removed_bench=True,prepare_manual_stop=prepare)
            report['stop_results'].append(result);last_stop=time.monotonic();persist()
        # Never exit while a parent arm RPC may still arrive. Dead parent is a
        # bounded grace case; the independent hard deadline stays active.
        finish=bool(lease.get('done') and not lease.get('rpc_busy'))
        orphan_done=(not alive and now-start>20 and (first_armed is None or now-first_armed>8))
        if armed_state() is False and (finish or orphan_done):
            if not alive:
                try:
                    mode(base_mode=0,custom_mode='MANUAL')
                    ownership(value=False)
                except Exception as exc:report['release_error']=str(exc)
            report.update(status='stopped',final_armed=False);persist();return
        time.sleep(.03)


if __name__=='__main__':main()
