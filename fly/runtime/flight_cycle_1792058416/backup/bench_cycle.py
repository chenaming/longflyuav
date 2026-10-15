#!/usr/bin/env python3
"""One NX lifecycle per bench run. Host model service is never stopped."""
import argparse,json,os,signal,socket,subprocess,sys,time
from pathlib import Path

BASE=Path(__file__).resolve().parent
ROOT=BASE.parent


def run_cycle(backend,check_only=False):
    backend.stop_stack()
    error=None
    try:
        backend.start_stack()
        backend.wait_ready()
        backend.infer()
        if not check_only:
            backend.model_chain()
            backend.precheck()
            backend.motor()
    except BaseException as exc:
        error=exc
    try:backend.stop_stack()
    except BaseException as exc:
        backend.keep_for_takeover(str(exc))
        if error is None:error=exc
    if error is not None:raise error


class Backend:
    def __init__(self,instruction):
        self.instruction=instruction
        self.report={'instruction':instruction,'started_unix':time.time(),'events':[]}

    def event(self,name,**data):
        print(name,flush=True)
        self.report['events'].append(dict(event=name,unix=time.time(),**data))
        self.report['updated_unix']=time.time()
        path=BASE/'runtime/bench_cycle_report.json';tmp=path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.report,ensure_ascii=False,indent=2));tmp.replace(path)

    def command(self,args,timeout=None):
        subprocess.run(args,check=True,timeout=timeout)

    @staticmethod
    def listening(port):
        try:
            with socket.create_connection(('127.0.0.1',port),timeout=.3):return True
        except OSError:return False

    def stop_stack(self):
        import fcntl
        # Serialise against both flight and bench scripts while checking and stopping.
        locks=[]
        for path in (BASE/'runtime/closed_loop_flight.lock',ROOT/'runtime/vla/test.lock'):
            handle=path.open('a');locks.append(handle)
            try:fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:raise RuntimeError('另一个控制/测试任务正在运行，不关闭节点。')
        try:
            if self.listening(11311):
                self.command([sys.executable,str(BASE/'stack_ground_guard.py')],timeout=9)
                self.command([sys.executable,str(ROOT/'vla_stack.py'),'stop'],timeout=32)
                if self.listening(11311):raise RuntimeError('存在非本启动器管理的 ROS 主节点，未强制清理。')
            # Stop only private relay/input clients from this project; never pkill ROS globally.
            for proc in Path('/proc').iterdir():
                if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
                try:
                    argv=(proc/'cmdline').read_bytes().split(b'\0')
                    if len(argv)<2 or b'python' not in Path(os.fsdecode(argv[0])).name.encode():continue
                    script_index=1
                    while script_index<len(argv) and argv[script_index] in (b'-u',b'-B'):
                        script_index+=1
                    if script_index>=len(argv) or argv[script_index].startswith(b'-'):continue
                    script=Path(os.fsdecode(argv[script_index]))
                    if not script.is_absolute():script=(proc/'cwd').resolve()/script
                    if script.resolve() not in (BASE/'relay.py',BASE/'fly_cli.py'):continue
                    pid=int(proc.name);os.kill(pid,signal.SIGINT)
                    deadline=time.monotonic()+2
                    while time.monotonic()<deadline and (proc/'cmdline').exists():
                        if not (proc/'cmdline').read_bytes():break
                        time.sleep(.05)
                    if (proc/'cmdline').exists() and (proc/'cmdline').read_bytes()==b'\0'.join(argv):os.kill(pid,signal.SIGTERM)
                except (OSError,ProcessLookupError):pass
            deadline=time.monotonic()+3
            while time.monotonic()<deadline and any(self.port_bound(p) for p in (11311,8088,15017)):
                time.sleep(.1)
            if any(self.port_bound(p) for p in (11311,8088,15017)):
                raise RuntimeError('11311、8088 或 15017 仍被占用，未启动重复服务。')
            self.event('NX 本轮节点已关闭；主机模型服务保持运行')
        finally:
            for handle in locks:handle.close()

    def start_stack(self):
        # Let NTP settle on boot before ROS/MAVROS establish their time bases.
        deadline=time.monotonic()+30
        while time.time()<1700000000 and time.monotonic()<deadline:time.sleep(.25)
        if time.time()<1700000000:raise RuntimeError('NX 时间未同步；未启动 ROS，请先检查系统时间/NTP。')
        self.event('启动 NX：ROS、Odin、MAVROS、定位转发、桥接、监控和推理转发')
        self.command([sys.executable,str(ROOT/'vla_stack.py'),'start'],timeout=8)

    @staticmethod
    def port_bound(port):
        for name in ('/proc/net/tcp','/proc/net/tcp6'):
            for line in Path(name).read_text().splitlines()[1:]:
                fields=line.split()
                if fields[3]=='0A' and int(fields[1].rsplit(':',1)[1],16)==port:return True
        return False

    def wait_ready(self):
        deadline=time.monotonic()+75
        path=ROOT/'runtime/vla/status.json'
        while time.monotonic()<deadline:
            try:
                status=json.loads(path.read_text())
                if (status.get('ready') and 0<=time.time()-status.get('updated_unix',0)<3
                        and not status.get('fcu',{}).get('armed',True)):
                    self.event('新一轮服务就绪',run_id=status['run_id']);return
            except (OSError,ValueError):pass
            time.sleep(.2)
        raise RuntimeError('启动后未获得新鲜定位和模型预览；请查看 runtime/vla 日志。')

    def infer(self):
        self.command(['bash',str(BASE/'start_fly.sh'),'--once',self.instruction],timeout=42)
        self.command([sys.executable,str(BASE/'verify_vision_forwarding.py')],timeout=12)
        self.event('当前指令的新鲜推理、Odin→飞控位置转发检查通过')

    def model_chain(self):
        self.event('自动验证本轮 8 个未解锁模型传递样本')
        self.command(['bash',str(BASE/'test_model_chain.sh')],timeout=55)

    def precheck(self):
        self.command([sys.executable,str(BASE/'motor_precheck.py'),'--allow-offboard-throttle'],timeout=25)

    def motor(self):
        self.event('启动 30 秒拆桨台架；切模式时交还遥控器')
        self.command([sys.executable,str(BASE/'bench60.py'),'--props-removed','--fixed'])

    def keep_for_takeover(self,reason):
        self.event('未满足关闭条件，保留节点供手控接管/检查',reason=reason)


def main():
    import fcntl
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--props-removed',action='store_true')
    parser.add_argument('--fixed',action='store_true')
    parser.add_argument('--check-only',action='store_true',help='只验证整套启停、推理与位置转发，不切模式、不解锁')
    parser.add_argument('--instruction')
    args=parser.parse_args()
    if not args.check_only and not (args.props_removed and args.fixed):parser.error('电机台架需要 --props-removed --fixed')
    (BASE/'runtime').mkdir(exist_ok=True)
    with (BASE/'runtime/bench_cycle.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('已有整轮测试启动器运行，请等待该轮结束。')
        config=json.loads((ROOT/'vla_config.json').read_text())
        instruction=(args.instruction or config.get('instruction','fly to the chair')).strip()
        if not instruction:parser.error('指令不能为空')
        backend=Backend(instruction)
        try:run_cycle(backend,args.check_only)
        except BaseException as exc:
            backend.event('本轮未完整通过',error=str(exc));raise SystemExit(1)
        backend.event('本轮完成并已关闭 NX 节点',check_only=args.check_only)


if __name__=='__main__':main()
