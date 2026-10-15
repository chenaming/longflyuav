"""ROS-independent ordinary shutdown policy: confirm LAND, ground, disarm."""


class NormalShutdown:
    def __init__(self,now,timeout=12.,ground_seconds=2.):
        self.started=now;self.timeout=timeout;self.ground_seconds=ground_seconds
        self.phase='LAND_REQUEST';self.reason='requesting AUTO.LAND with hold stream alive'
        self.ground_since=None;self.last_mode=-1e9;self.last_disarm=-1e9
        self.disarm_requests=0;self.auto_disarmed=False;self.land_confirmed=False

    def step(self,now,s,rpc_ready=True):
        if self.phase in ('DONE','FAILED','TAKEOVER'):return False,None
        if not s['fresh'] or not s['connected']:
            self.phase='FAILED';self.reason='FCU state stale during ordinary shutdown';return False,None
        if s['armed'] and s['mode'] not in ('OFFBOARD','AUTO.LAND'):
            self.phase='TAKEOVER';self.reason='pilot/PX4 selected '+s['mode'];return False,None
        ground=bool(s.get('land_fresh') and s.get('landed'))
        if not s['armed'] and ground:
            self.phase='DONE';self.auto_disarmed=self.disarm_requests==0
            self.reason='fresh ground and disarmed state confirmed';return False,None
        if now-self.started>self.timeout:
            self.phase='FAILED';self.reason='ordinary landing/disarm confirmation timed out';return False,None
        if not s['armed']:return False,None
        if s['mode']=='OFFBOARD':
            self.phase='LAND_REQUEST'
            if rpc_ready and now-self.last_mode>=1:
                self.last_mode=now;return True,('mode','AUTO.LAND')
            return True,None
        self.land_confirmed=True;self.phase='WAIT_GROUND'
        if ground:
            if self.ground_since is None:self.ground_since=now
            if now-self.ground_since>=self.ground_seconds:
                self.phase='WAIT_DISARM'
                if rpc_ready and now-self.last_disarm>=1 and self.disarm_requests<3:
                    self.last_disarm=now;self.disarm_requests+=1
                    return False,('arm',False)
        else:self.ground_since=None
        return False,None
