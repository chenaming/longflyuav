"""Disarm-only cleanup; retain results even if a ROS service raises."""
import time


def bench_disarm(arm, force, armed_state, *, propellers_removed_bench=False,
                  sleep=time.sleep):
    result = {'normal_response': None, 'force_disarm_used': False}

    def stopped():
        try:
            return armed_state() is False
        except Exception as exc:
            result['state_error'] = str(exc)
            return False

    if stopped():
        result.update(already_disarmed=True, disarmed_confirmed=True)
        return result
    try:
        response = arm(value=False)
        result['normal_response'] = {'success': response.success, 'result': response.result}
    except Exception as exc:
        result['normal_error'] = str(exc)
    # A service error does not say whether the FCU actually stopped.
    for _ in range(15):
        if stopped():
            result['disarmed_confirmed'] = True
            return result
        sleep(.1)
    if not propellers_removed_bench:
        result.update(disarmed_confirmed=False,
                      error='force-disarm requires explicit propeller-removed bench context')
        return result
    result['force_disarm_used'] = True
    try:
        response = force(broadcast=False, command=400, confirmation=0,
                         param1=0., param2=21196., param3=0., param4=0.,
                         param5=0., param6=0., param7=0.)
        result['force_response'] = {'success': response.success, 'result': response.result}
    except Exception as exc:
        result['force_error'] = str(exc)
    for _ in range(20):
        if stopped():
            result['disarmed_confirmed'] = True
            return result
        sleep(.1)
    result['disarmed_confirmed'] = stopped()
    return result
