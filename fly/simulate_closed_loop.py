#!/usr/bin/env python3
"""Deterministic software harness, NOT PX4 SITL and NOT a flight validation."""
import json
from pathlib import Path
from flight_core import Config,Mission,norm


def simulate():
    epoch=1792036000.;dt=.05;pose=[0.,0.,.3,0.]
    m=Mission(Config(allow_paired=True),'fly to the chair',0,epoch,pose)
    state=dict(fresh=True,connected=True,armed=False,mode='MANUAL',landed=True)
    speed=0.;events=[];last_phase=None;request=0;sp=None
    for step in range(1,1601):
        now=step*dt;pred=None
        if step%5==0:
            request+=1
            dx=.25 if m.navigation_start is None or now-m.navigation_start<8 else 0.
            pred=dict(run_id='sim',epoch=1,instruction='fly to the chair',target_frame='map',request_id=request,
                      observation_stamp=epoch+now,latest_only=True,coordinate_mode='paired_estimate',
                      target_xyz_m=[pose[0]+dx,pose[1],pose[2]],target_yaw_rad=pose[3],
                      observed_xyz_m=pose[:3],observed_yaw_rad=pose[3],action_ori_cm_rad=[dx*100,0,0,0])
        sp,action=m.step(now,epoch+now,pose,speed,0.,state,pred)
        if m.phase!=last_phase:
            events.append(dict(t=round(now,2),phase=m.phase,reason=m.reason));last_phase=m.phase
        if action:
            events.append(dict(t=round(now,2),rpc=list(action)))
            if action[0]=='mode':state['mode']=action[1]
            elif action[0]=='arm':state['armed']=action[1]
        old=list(pose)
        if state['armed'] and state['mode']=='OFFBOARD' and sp:
            for i in range(3):
                pose[i]+=(sp[i]-pose[i])*.30
                # Resolve the toy plant's asymptotic approach at numeric precision.
                if abs(sp[i]-pose[i])<1e-9:pose[i]=sp[i]
            if pose[2]>.34:state['landed']=False
        if state['armed'] and state['mode']=='AUTO.LAND':
            pose[2]=max(.3,pose[2]-.2*dt)
            state['landed']=pose[2]<=.300001
        speed=norm([a-b for a,b in zip(pose[:3],old[:3])])/dt
        if m.phase in ('DONE','ABORTED','TAKEOVER'):break
    assert m.phase=='DONE',events
    assert m.arrival_inferred and m.accepted_predictions>8
    assert not state['armed'] and state['landed']
    assert any(e.get('phase')=='WAIT_NEW_IMAGE' for e in events)
    return dict(passed=True,kind='software policy simulation, not PX4 SITL',events=events,
                final_state=state,accepted_predictions=m.accepted_predictions,
                arrival_is_heuristic=True,physical_flight_validated=False)


if __name__=='__main__':
    report=simulate();folder=Path(__file__).resolve().parent/'runtime';folder.mkdir(exist_ok=True)
    path=folder/'flight_simulation_report.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2));print('报告：'+str(path))
