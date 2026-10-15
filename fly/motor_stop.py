"""Disarm-only cleanup; retain results even if a ROS service raises."""
import time


def bench_disarm(arm, force, armed_state, *, propellers_removed_bench=False,
                  prepare_manual_stop=None, sleep=time.sleep):
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
    if propellers_removed_bench and prepare_manual_stop is not None:
        try:
            result['manual_stop_preparation'] = prepare_manual_stop()
        except Exception as exc:
            result['manual_stop_error'] = str(exc)
    try:
        response = arm(value=False)
        result['normal_response'] = {'success': response.success, 'result': response.result}
    except Exception as exc:
        result['normal_error'] = str(exc)
    # A service error does not say whether the FCU actually stopped.
    for index in range(15):
        if stopped():
            result['disarmed_confirmed'] = True
            return result
        sleep(.1)
        # Low-throttle MANUAL lets the normal landed check settle. All retries
        # are disarm requests, still bounded to the existing 1.5s wait.
        if (index in (4,9,14)
                and result.get('manual_stop_preparation',{}).get('mode_sent')
                and not (result.get('normal_response') or {}).get('success')):
            try:
                response = arm(value=False)
                result.setdefault('normal_retry_responses',[]).append({'success':response.success,'result':response.result})
                result['normal_response'] = {'success':response.success,'result':response.result}
            except Exception as exc:
                result['normal_error'] = str(exc)
    if stopped():
        result['disarmed_confirmed'] = True
        return result
    # The last ordinary retry can be accepted at the end of the 1.5s loop.
    # MAVROS State is delivered asynchronously (typically the FCU heartbeat).
    # ACK success alone is insufficient, but forcing immediately can race the
    # already accepted disarm. Allow one bounded state-confirmation interval.
    if (result.get('normal_response') or {}).get('success'):
        result['normal_ack_state_wait_s'] = 0.
        for _ in range(12):
            sleep(.1)
            result['normal_ack_state_wait_s'] = round(result['normal_ack_state_wait_s'] + .1, 1)
            if stopped():
                result['disarmed_confirmed'] = True
                return result
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
