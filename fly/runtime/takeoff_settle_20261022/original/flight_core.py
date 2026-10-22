"""ROS-independent, deterministic mission policy. No service calls in this module."""
from dataclasses import dataclass
import math


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def norm(v):
    return math.sqrt(sum(x*x for x in v))


@dataclass
class Config:
    height: float = 1.0
    speed: float = 0.20
    yaw_speed: float = math.radians(10)
    settle_seconds: float = 2.0
    observation_age: float = 2.0
    prediction_gap: float = 2.0
    inference_failure_land: float = 5.0
    max_xy_step: float = 0.35
    max_z_step: float = 0.05
    max_yaw_step: float = math.radians(8)
    radius: float = 3.0
    min_nav_height: float = 0.5
    max_height: float = 1.5
    max_navigation: float = 60.0
    arrival_step: float = 0.03
    arrival_yaw: float = math.radians(3)
    arrival_speed: float = 0.08
    arrival_yaw_speed: float = math.radians(3)
    arrival_count: int = 8
    arrival_seconds: float = 2.0
    arrival_gap: float = 1.0
    min_navigation: float = 5.0
    allow_paired: bool = False

    def validate(self):
        for k, v in vars(self).items():
            if k != 'allow_paired' and (not math.isfinite(v) or v <= 0):
                raise ValueError('invalid positive setting: ' + k)
        if not self.min_nav_height < self.height <= self.max_height:
            raise ValueError('min_nav_height < height <= max_height required')
        if self.arrival_count < 2 or self.prediction_gap >= self.inference_failure_land:
            raise ValueError('invalid arrival count / timeout ordering')


class Mission:
    def __init__(self, cfg, instruction, now, ros_now, pose):
        cfg.validate()
        self.cfg, self.instruction = cfg, instruction
        self.phase, self.reason = 'WAIT_INITIAL', 'waiting for a fresh model response'
        self.started, self.entered, self.start_stamp = now, now, ros_now
        self.home = tuple(pose[:3]); self.home_yaw = pose[3]
        self.sp = list(pose); self.target = list(pose)
        self.identity = None
        self.last_image = -1.; self.last_request = -1
        self.last_prediction = None; self.last_valid_image = None
        self.navigation_stamp = float('inf'); self.navigation_start = None
        self.small = []; self.stable_since = None
        self.takeoff_stable = None; self.ground_since = None
        self.arm_requested = False; self.disarm_requested = False
        self.service_at = -1e9
        self.last_tick = now
        self.last_ros = ros_now
        self.arrival_inferred = False
        self.accepted_predictions = 0
        self.rejected_predictions = 0

    def transition(self, phase, now, reason):
        self.phase, self.entered, self.reason = phase, now, reason
        self.service_at = -1e9

    def land(self, now, pose, reason, arrival=False):
        if self.phase in ('LAND_REQUEST', 'LANDING', 'DONE', 'TAKEOVER', 'ABORTED'):
            return
        self.sp = list(pose); self.target = list(pose)
        self.small.clear(); self.arrival_inferred = arrival
        self.transition('LAND_REQUEST', now, reason)

    def accept(self, p, now, ros_now, pose):
        c = self.cfg
        try:
            stamp = float(p['observation_stamp'])
            request = int(p['request_id'])
            identity = (p['run_id'], p['epoch'], p['instruction'], p['target_frame'])
            if identity[2:] != (self.instruction, 'map'):
                raise ValueError('instruction or frame changed')
            if self.identity is not None and identity != self.identity:
                raise ValueError('bridge run / reference epoch changed')
            if not c.allow_paired and p.get('coordinate_mode') != 'tf':
                raise ValueError('paired coordinate estimate requires explicit opt-in')
            if not p.get('latest_only'):
                raise ValueError('bridge does not certify latest-image sampling')
            if stamp <= self.last_image or request <= self.last_request:
                return False
            if not 0 <= ros_now - stamp <= c.observation_age or stamp < self.start_stamp:
                raise ValueError('old image')
            xyz = list(map(float, p['target_xyz_m'])); yaw = float(p['target_yaw_rad'])
            raw = list(map(float, p['action_ori_cm_rad']))
            observed = list(map(float, p['observed_xyz_m']))
            observed_yaw = float(p['observed_yaw_rad'])
            if len(xyz) != 3 or len(raw) != 4 or len(observed) != 3:
                raise ValueError('invalid vector size')
            if not all(math.isfinite(x) for x in xyz + [yaw] + raw + observed + [observed_yaw]):
                raise ValueError('nonfinite prediction')
            for origin, origin_yaw in ((pose[:3], pose[3]), (observed, observed_yaw)):
                delta = [a-b for a,b in zip(xyz, origin)]
                if math.hypot(*delta[:2]) > c.max_xy_step or abs(delta[2]) > c.max_z_step or abs(wrap(yaw-origin_yaw)) > c.max_yaw_step:
                    raise ValueError('model step outside limits')
            self.identity = identity
            self.last_image, self.last_request = stamp, request
            if self.phase == 'WAIT_INITIAL':
                self.transition('PRESTREAM', now, 'fresh initial prediction received; holding ground position')
                return True  # Ground prediction is a gate, never a flight destination.
            if self.phase not in ('NAVIGATE', 'WAIT_NEW_IMAGE', 'HOLD_STALE') or stamp <= self.navigation_stamp:
                return False
            relative = [a-b for a,b in zip(xyz, self.home)]
            if math.hypot(*relative[:2]) > c.radius or not c.min_nav_height <= relative[2] <= c.max_height:
                raise ValueError('model target outside flight volume')
            if self.phase != 'NAVIGATE':
                self.transition('NAVIGATE', now, 'following fresh model targets')
            self.target = xyz + [yaw]
            self.last_prediction, self.last_valid_image = now, stamp
            self.accepted_predictions += 1
            small = norm([x/100 for x in raw[:3]]) <= c.arrival_step and abs(raw[3]) <= c.arrival_yaw
            small = small and norm([a-b for a,b in zip(xyz, observed)]) <= c.arrival_step
            small = small and abs(wrap(yaw-observed_yaw)) <= c.arrival_yaw
            small = small and norm([a-b for a,b in zip(xyz, pose[:3])]) <= c.arrival_step
            small = small and abs(wrap(yaw-pose[3])) <= c.arrival_yaw
            if self.small and (stamp - self.small[-1][0] > c.arrival_gap or now-self.small[-1][1] > c.arrival_gap):
                self.small.clear()
            if small:
                self.small.append((stamp, now))
                self.small = self.small[-200:]
            else:
                self.small.clear()
            return True
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            self.rejected_predictions += 1; self.small.clear()
            self.reason = 'prediction rejected: ' + str(exc)
            return False

    def step(self, now, ros_now, pose, speed, yaw_rate, state, prediction=None, fault=None, stop=False):
        """state contains fresh, connected, armed, mode, landed. Return SP + optional RPC intent."""
        c = self.cfg
        dt = min(max(now-self.last_tick, 0), .1); elapsed = now-self.last_tick
        self.last_tick = now
        jump = abs((ros_now-self.last_ros)-elapsed) > .5
        self.last_ros = ros_now
        action = None
        terminal = ('DONE', 'TAKEOVER', 'ABORTED')
        if self.phase in terminal:
            return None, None
        if (state['fresh'] and state['armed'] and not self.arm_requested
                and self.phase in ('WAIT_INITIAL','PRESTREAM','ENTER_OFFBOARD')):
            self.transition('TAKEOVER', now, 'armed outside this mission; no automatic takeover')
            return None, None
        if (not state['armed'] and self.phase in ('WAIT_INITIAL','PRESTREAM','ENTER_OFFBOARD','ARM')
                and norm([a-b for a,b in zip(pose[:3],self.home)]) > .08):
            self.transition('ABORTED',now,'vehicle moved before takeoff; re-establish starting position')
            return None, None
        if jump:
            fault = 'ROS clock jumped'
        if fault or stop:
            if state['armed']:
                self.land(now, pose, fault or 'operator requested landing')
            elif self.phase not in ('LAND_REQUEST','LANDING'):
                self.transition('ABORTED', now, fault or 'operator stopped before arming')
                return None, None
        expected_offboard = self.phase in ('ARM', 'TAKEOFF', 'WAIT_NEW_IMAGE', 'NAVIGATE', 'HOLD_STALE', 'HOLD_LAND_FAILED')
        if state['fresh'] and expected_offboard and state['mode'] != 'OFFBOARD':
            self.transition('TAKEOVER', now, 'PX4 / operator left OFFBOARD; no automatic re-entry')
            return None, None
        if self.phase in ('TAKEOFF','WAIT_NEW_IMAGE','NAVIGATE','HOLD_STALE') and state['fresh'] and not state['armed']:
            self.transition('ABORTED', now, 'unexpected disarm; no re-arm')
            return None, None
        if self.phase not in ('LAND_REQUEST','LANDING') and state['armed']:
            rel = [a-b for a,b in zip(pose[:3],self.home)]
            if math.hypot(*rel[:2]) > c.radius or rel[2] > c.max_height + .15:
                self.land(now, pose, 'vehicle outside flight volume')
        if prediction is not None and not fault and not stop:
            self.accept(prediction, now, ros_now, pose)
        if self.phase == 'WAIT_INITIAL':
            if now-self.started > 20:
                self.transition('ABORTED', now, 'no fresh initial prediction within 20 s')
        elif self.phase == 'PRESTREAM':
            if now-self.entered >= 2:
                self.transition('ENTER_OFFBOARD', now, 'requesting OFFBOARD')
        elif self.phase == 'ENTER_OFFBOARD':
            if state['mode'] == 'OFFBOARD' and state['fresh']:
                self.transition('ARM', now, 'OFFBOARD observed; requesting normal arm')
            elif now-self.entered > 6:
                self.transition('ABORTED', now, 'OFFBOARD not confirmed; no arming')
            elif now-self.service_at >= 1:
                action = ('mode','OFFBOARD'); self.service_at=now
        elif self.phase == 'ARM':
            if state['armed'] and state['fresh']:
                self.transition('TAKEOFF', now, 'climbing 1 m relative to starting position')
                self.target = list(self.home[:2]) + [self.home[2]+c.height,self.home_yaw]
            elif now-self.entered > 6:
                self.transition('ABORTED', now, 'normal arm not confirmed; no retry loop')
            elif not self.arm_requested and 0 <= ros_now-self.last_image <= c.observation_age:
                action=('arm',True); self.arm_requested=True
        elif self.phase == 'TAKEOFF':
            near = norm([a-b for a,b in zip(pose[:3],self.target[:3])]) <= .08
            stable = near and speed <= c.arrival_speed and abs(yaw_rate) <= c.arrival_yaw_speed
            self.takeoff_stable = (self.takeoff_stable if self.takeoff_stable is not None else now) if stable else None
            if self.takeoff_stable is not None and now-self.takeoff_stable >= c.settle_seconds:
                self.navigation_stamp=ros_now; self.navigation_start=now
                self.sp=list(pose); self.target=list(pose); self.small.clear()
                self.transition('WAIT_NEW_IMAGE',now,'hover settled; discarding all pre-hover images/results')
            elif now-self.entered > 60:
                self.land(now,pose,'takeoff did not settle within 20 s')
        elif self.phase in ('WAIT_NEW_IMAGE','NAVIGATE','HOLD_STALE'):
            since = now-(self.last_prediction if self.last_prediction is not None else self.navigation_start)
            image_age = ros_now-self.last_valid_image if self.last_valid_image is not None else float('inf')
            if since > c.inference_failure_land:
                self.land(now,pose,'inference unavailable for 5 s; abort landing, not arrival')
            elif now-self.navigation_start > c.max_navigation:
                self.land(now,pose,'navigation time limit; abort landing, not arrival')
            elif self.phase == 'NAVIGATE' and (since > c.prediction_gap or image_age > c.observation_age):
                self.sp=list(pose); self.target=list(pose); self.small.clear()
                self.transition('HOLD_STALE',now,'inference stale; hold a fixed position')
            if self.small and (now-self.small[-1][1] > c.arrival_gap or ros_now-self.small[-1][0] > c.observation_age):
                self.small.clear()
            stable = speed <= c.arrival_speed and abs(yaw_rate) <= c.arrival_yaw_speed
            if not stable:
                self.small.clear(); self.stable_since=None
            elif self.stable_since is None:
                self.stable_since=now
            if (self.phase == 'NAVIGATE' and len(self.small) >= c.arrival_count
                and self.small[-1][0]-self.small[0][0] >= c.arrival_seconds
                and self.small[-1][1]-self.small[0][1] >= c.arrival_seconds
                and self.stable_since is not None and now-self.stable_since >= c.arrival_seconds
                and now-self.navigation_start >= c.min_navigation):
                self.land(now,pose,'consecutive small actions + stable aircraft (heuristic arrival)',arrival=True)
        elif self.phase == 'LAND_REQUEST':
            if state['fresh'] and state['mode'] == 'AUTO.LAND':
                self.transition('LANDING',now,self.reason)
            elif state['fresh'] and state['mode'] != 'OFFBOARD':
                self.transition('TAKEOVER',now,'another flight mode took control during landing request')
            elif now-self.entered > 8:
                self.transition('HOLD_LAND_FAILED',now,'AUTO.LAND not confirmed; holding, pilot intervention required')
            elif now-self.service_at >= 1:
                action=('mode','AUTO.LAND'); self.service_at=now
        elif self.phase == 'LANDING':
            if state['fresh'] and not state['armed'] and state['landed']:
                self.transition('DONE',now,self.reason)
            elif state['fresh'] and state['mode'] != 'AUTO.LAND':
                self.transition('TAKEOVER',now,'landing mode exited; operator/PX4 owns control')
            elif state['fresh'] and state['landed']:
                if self.ground_since is None: self.ground_since=now
                if now-self.ground_since >= 2 and not self.disarm_requested:
                    action=('arm',False);self.disarm_requested=True
            else:
                self.ground_since=None
        if self.phase in ('TAKEOFF','NAVIGATE'):
            # Bound commanded speed and avoid running far ahead of the measured vehicle.
            delta=[a-b for a,b in zip(self.target[:3],self.sp[:3])]
            distance=norm(delta);scale=min(1,c.speed*dt/max(distance,1e-9))
            candidate=[a+b*scale for a,b in zip(self.sp[:3],delta)]
            ahead=[a-b for a,b in zip(candidate,pose[:3])];d=norm(ahead)
            if d > .20: candidate=[a+b*.20/d for a,b in zip(pose[:3],ahead)]
            self.sp[:3]=candidate
            dyaw=wrap(self.target[3]-self.sp[3]);limit=c.yaw_speed*dt
            self.sp[3]=wrap(self.sp[3]+max(-limit,min(limit,dyaw)))
        publish = self.phase not in terminal + ('LANDING',)
        return list(self.sp) if publish else None, action
