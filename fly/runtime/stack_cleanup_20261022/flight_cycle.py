#!/usr/bin/env python3
"""Fresh NX lifecycle for formal flight; default checks only and never arms."""
import argparse,json,sys
from bench_cycle import Backend,BASE,ROOT


def run_flight_cycle(backend,check_only=True):
    backend.stop_stack()
    error=None
    try:
        backend.start_stack();backend.wait_ready();backend.infer()
        backend.flight(check_only)
    except BaseException as exc:error=exc
    try:
        backend.wait_ground(check_only)
        backend.stop_stack()
    except BaseException as exc:
        backend.keep_for_takeover(str(exc))
        if error is None:error=exc
    if error is not None:raise error


class FlightBackend(Backend):
    report_filename='flight_cycle_report.json'

    def __init__(self,instruction,allow_paired):
        super().__init__(instruction);self.allow_paired=allow_paired;self.execute_started=False

    def flight(self,check_only):
        args=[sys.executable,str(BASE/'closed_loop_flight.py'),
              '--check-only' if check_only else '--execute','--instruction',self.instruction]
        if self.allow_paired:args.append('--allow-paired-estimate')
        self.event('执行只读预检，不解锁' if check_only else '执行正式起飞、模型导航与降落；遥控器切模式可接管')
        self.execute_started=not check_only
        self.command(args)

    def wait_ground(self,check_only):
        # Keep the positioning chain alive after pilot takeover or AUTO.LAND.
        if not self.listening(11311):return
        timeout=5 if check_only or not self.execute_started else 300
        self.command([sys.executable,str(BASE/'stack_ground_guard.py'),'--timeout',str(timeout)],timeout=timeout+4)


def main():
    import fcntl
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--check-only',action='store_true')
    mode.add_argument('--stop-only',action='store_true',help='落地上锁后只关闭 NX 节点')
    parser.add_argument('--allow-paired-estimate',action='store_true')
    parser.add_argument('--instruction')
    args=parser.parse_args()
    (BASE/'runtime').mkdir(exist_ok=True)
    with (BASE/'runtime/bench_cycle.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有整轮飞行或台架任务，请等待其结束。')
        config=json.loads((ROOT/'vla_config.json').read_text())
        instruction=(args.instruction or config.get('instruction','fly to the chair')).strip()
        if not instruction:parser.error('指令不能为空')
        backend=FlightBackend(instruction,args.allow_paired_estimate)
        try:
            if args.stop_only:backend.stop_stack()
            else:run_flight_cycle(backend,check_only=not args.execute)
        except BaseException as exc:
            backend.event('本轮未完整通过',error=str(exc));raise SystemExit(1)
        backend.event('本轮完成，NX 节点已关闭',check_only=not args.execute)


if __name__=='__main__':main()
