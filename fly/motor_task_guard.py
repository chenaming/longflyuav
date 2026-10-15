"""Match the captured operator task before and during a finite motor bench."""


def same_task(prediction, expected, active_instruction, active_run_id):
    instruction = expected.get('instruction')
    run_id = expected.get('run_id')
    return bool(instruction and run_id
                and prediction.get('instruction') == instruction == active_instruction
                and prediction.get('run_id') == run_id == active_run_id
                and prediction.get('target_frame') == 'map')


def rc_is_ready(channels, *, require_low_throttle=True):
    return (len(channels) >= 9 and channels[8] < 1300
            and (not require_low_throttle or channels[2] < 1100))
