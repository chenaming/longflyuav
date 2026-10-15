"""Ground-only timed stages; never a real takeoff/arrival policy."""
import math


def wrap(x):
    return (x+math.pi)%(2*math.pi)-math.pi


def watchdog_reasons(now,start,armed_since,alive,lease,lease_at,state,state_at,rc,rc_at,initial_ch6):
    reasons=[]
    if not alive:reasons.append('parent exited')
    if lease.get('stop'):reasons.append('parent requested stop')
    if now-start>85:reasons.append('overall 85s limit')
    if armed_since is not None:
        if now-armed_since>65:reasons.append('armed 65s limit')
        if now-lease_at>.6:reasons.append('policy heartbeat stale')
        if not state or not state.connected or now-state_at>1.5:reasons.append('FCU state stale')
        elif state.armed and state.mode!='OFFBOARD':reasons.append('left OFFBOARD')
        if not rc or now-rc_at>.75 or len(rc.channels)<9 or rc.rssi==0:reasons.append('RC stale')
        elif (rc.channels[2]>=1100 or rc.channels[8]>=1300 or initial_ch6 is None
              or abs(rc.channels[5]-initial_ch6)>200):reasons.append('RC stop')
    return reasons


class Bench60:
    duration = 60.
    def __init__(self, home, identity):
        self.home=list(home);self.identity=tuple(identity)
        self.started=None;self.cutoff=None;self.last_image=-1.;self.last_request=-1
        self.samples=[];self.phase='PRESTREAM';self.sp=list(home)

    def begin(self, now):
        if self.started is not None:raise ValueError('bench already started')
        self.started=now

    def step(self, now, ros_now):
        elapsed=now-self.started
        if elapsed<10:self.phase='SIMULATED_TAKEOFF'
        elif elapsed<12:self.phase='SIMULATED_HOVER'
        elif elapsed<60:
            if self.cutoff is None:self.cutoff=ros_now
            self.phase='MODEL_LOOP'
        else:self.phase='STOPPING'
        return self.phase

    def accept(self,p,now,ros_now):
        identity=(p.get('run_id'),p.get('epoch'),p.get('instruction'))
        if identity!=self.identity:raise ValueError('任务或参考原点改变')
        stamp=float(p['observation_stamp']);request=int(p['request_id'])
        if not p.get('latest_only') or p.get('target_frame')!='map':raise ValueError('缺少最新图像或map坐标保证')
        if not 0<=ros_now-stamp<=2:raise ValueError('模型图像已过期')
        if stamp<=self.last_image or request<=self.last_request:return False
        xyz=list(map(float,p['target_xyz_m']));yaw=float(p['target_yaw_rad'])
        raw=list(map(float,p['action_ori_cm_rad']))
        if len(xyz)!=3 or len(raw)!=4 or not all(math.isfinite(v) for v in xyz+[yaw]+raw):
            raise ValueError('模型坐标无效')
        self.last_image,self.last_request=stamp,request
        if self.phase!='MODEL_LOOP' or stamp<=self.cutoff:return False
        delta=[a-b for a,b in zip(xyz,self.home[:3])]
        scale=min(1.,.02/max(math.hypot(*delta[:2]),1e-9))
        delta=[delta[0]*scale,delta[1]*scale,max(-.003,min(.003,delta[2]))]
        dyaw=max(-math.radians(1),min(math.radians(1),wrap(yaw-self.home[3])))
        self.sp=[self.home[i]+delta[i] for i in range(3)]+[self.home[3]+dyaw]
        self.samples.append({'t':now,'request_id':request,'image_stamp':stamp,
            'age_s':ros_now-stamp,'raw_target_xyz_m':xyz,'raw_target_yaw_rad':yaw,
            'sent_xyz_yaw':list(self.sp)})
        return True
