"""Disarm-only cleanup for an explicitly confirmed propeller-removed bench.

The forced operation ALWAYS uses param1=0. It cannot arm the vehicle.
It must never be used for an airborne vehicle.
"""
import time


def bench_disarm(arm, force, armed_state, *, propellers_removed_bench=False,
                  sleep=time.sleep):
    result = {'normal_response': None, 'force_disarm_used': False}
    response = arm(value=False)
    result['normal_response'] = {'success': response.success, 'result': response.result}
    if response.success:
        for _ in range(15):
            if armed_state() is False:
                result['disarmed_confirmed'] = True
                return result
            sleep(.1)
    if armed_state() is False:
        result['disarmed_confirmed'] = True
        return result
    if not propellers_removed_bench:
        result['disarmed_confirmed'] = False
        result['error'] = 'force-disarm requires explicit propeller-removed bench context'
        return result
    response = force(broadcast=False, command=400, confirmation=0,
                     param1=0., param2=21196., param3=0., param4=0.,
                     param5=0., param6=0., param7=0.)
    result['force_disarm_used'] = True
    result['force_response'] = {'success': response.success, 'result': response.result}
    for _ in range(20):
        if armed_state() is False:
            result['disarmed_confirmed'] = True
            return result
        sleep(.1)
    result['disarmed_confirmed'] = armed_state() is False
    return result
