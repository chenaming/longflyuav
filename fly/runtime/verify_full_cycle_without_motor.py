"""Exercise real NX lifecycle, with passive checks and isolated test topics only."""
import sys,subprocess,fcntl
sys.path.insert(0,'/home/nvidia/agile/fly')
from bench_cycle import Backend,run_cycle


class Verification(Backend):
    def infer(self):
        self.command([sys.executable,'/home/nvidia/agile/fly/runtime/inspect_highfreq_timing.py'],timeout=9)
        command='g++ -std=c++14 /home/nvidia/agile/fly/runtime/test_odometry_bridge_freshness.cpp -I/home/nvidia/agile/agile_ws/src/uav_control_fsm/include -L/home/nvidia/agile/agile_ws/devel/lib -Wl,-rpath,/home/nvidia/agile/agile_ws/devel/lib -luav_odometry_bridge $(pkg-config --cflags --libs roscpp) -o /home/nvidia/agile/fly/runtime/test_odometry_bridge_freshness'
        self.command(['bash','-c',command],timeout=25)
        self.command(['/home/nvidia/agile/fly/runtime/test_odometry_bridge_freshness'],timeout=6)
        super().infer()
        self.event('位置转发实链路和隔离话题的位姿过期测试通过；未调用真实解锁/模式服务')


cycle_lock=open('/home/nvidia/agile/fly/runtime/bench_cycle.lock','a')
fcntl.flock(cycle_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
backend=Verification('fly to the chair')
run_cycle(backend,check_only=True)
assert not any(backend.port_bound(p) for p in (11311,8088,15017))
backend.event('整轮只读启停验证通过，11311/8088/15017 已释放')
