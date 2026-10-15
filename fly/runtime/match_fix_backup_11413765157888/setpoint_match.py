"""Match bounded position commands to the controller's actual smoothing rule."""
import math


def effective_commands(commands, smooth_enabled):
    goal = None
    result = []
    for t, entry in commands:
        target = list(entry['sent_xyz_m'])
        if (not smooth_enabled or goal is None or
                math.sqrt(sum((a-b)**2 for a,b in zip(target,goal))) > 1e-3):
            goal = target
        result.append((t,dict(entry,effective_xyz_m=list(goal))))
    return result


def match_setpoints(commands, setpoints, *, smooth_enabled):
    projected = effective_commands(commands,smooth_enabled)
    matches = []
    for point in setpoints:
        t = point['t']
        candidates = [entry for ts,entry in projected if t-.2 <= ts <= t+.02]
        def matches_entry(entry):
            yaw_error = (math.radians(entry['sent_yaw_deg'])-point['yaw_rad']+math.pi) % (2*math.pi)-math.pi
            return (max(abs(a-b) for a,b in zip(entry['effective_xyz_m'],point['xyz_m'])) < 1e-4
                    and abs(yaw_error) < 1e-4)
        matched = (point['source']=='/uav1/uav_control_fsm' and point['coordinate_frame']==1
                   and any(matches_entry(entry) for entry in candidates))
        matches.append(matched)
    return {'total':len(setpoints),'matched':sum(matches),'matches':matches,
            'ratio':sum(matches)/len(matches) if matches else 0.}
