"""Linux regression tests; only disposable dummy processes receive signals."""
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch

from stack_processes import Processes,alive,process,remaining,save
import bench_cycle


class ProcessTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        (self.root/'runtime/vla').mkdir(parents=True)
        self.children=[];self.tracked=Processes()

    def tearDown(self):
        self.tracked.stop((.1,.1,1.))
        for child in self.children:
            if child.poll() is None:child.kill()
            child.wait(timeout=3)
            if child.stdout:child.stdout.close()
        self.temp.cleanup()

    def launch(self,code):
        child=subprocess.Popen([sys.executable,'-u','-c',code],stdout=subprocess.PIPE,
                               stderr=subprocess.DEVNULL,text=True,start_new_session=True)
        self.children.append(child);self.tracked.add(child.pid)
        return child

    def test_detached_orphan_survives_parent_then_is_recovered_and_killed(self):
        code="import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);print('ready',flush=True);time.sleep(60)"
        parent=self.launch("import subprocess,sys,time;child=subprocess.Popen([sys.executable,'-u','-c',%r],start_new_session=True);print(child.pid,flush=True);time.sleep(60)"%code)
        child_pid=int(parent.stdout.readline());self.assertEqual(parent.stdout.readline().strip(),'ready')
        tracked=Processes();tracked.add(parent.pid)
        self.tracked.refresh()
        self.assertIn(child_pid,tracked.items)
        self.assertNotEqual(os.getpgid(parent.pid),os.getpgid(child_pid))
        save(self.root/'runtime/vla/owned_processes.json',[tracked])
        parent.terminate();parent.wait(timeout=3)
        recovered=remaining(self.root)
        self.assertIn(child_pid,recovered.items)
        orphan=recovered.items[child_pid]
        recovered.stop((.1,.1,1.))
        self.assertFalse(alive(orphan))

    def test_dead_parent_still_has_tracked_descendants_and_unrelated_survives(self):
        unrelated=self.launch("import time;print('ready');time.sleep(60)")
        self.assertEqual(unrelated.stdout.readline().strip(),'ready')
        parent=self.launch("import subprocess,sys,time;child=subprocess.Popen([sys.executable,'-u','-c',\"import time;print('ready');time.sleep(60)\"],start_new_session=True);print(child.pid,flush=True);time.sleep(60)")
        child_pid=int(parent.stdout.readline());self.assertEqual(parent.stdout.readline().strip(),'ready')
        group=Processes();group.add(parent.pid);self.tracked.refresh()
        parent.terminate();parent.wait(timeout=3)
        group.stop((.1,.1,1.))
        self.assertIsNone(process(child_pid));self.assertIsNone(unrelated.poll())

    def test_pid_identity_mismatch_is_never_signalled(self):
        child=self.launch("import time;print('ready');time.sleep(60)")
        child.stdout.readline()
        identity=process(child.pid);identity['start']+=1
        group=Processes([identity]);group.stop((.1,.1,.1))
        self.assertIsNone(child.poll())

    def test_legacy_odin_executable_is_found_without_manifest(self):
        executable=self.root/'odin_ws/devel/lib/odin_ros_driver/host_sdk_sample'
        executable.parent.mkdir(parents=True);shutil.copy2('/bin/sleep',str(executable))
        child=subprocess.Popen([str(executable),'60'],start_new_session=True)
        self.children.append(child)
        group=remaining(self.root)
        self.assertIn(child.pid,group.items)
        group.stop((.1,.1,1.));child.wait(timeout=3)

    def test_other_boot_manifest_is_ignored(self):
        child=self.launch("import time;print('ready');time.sleep(60)");child.stdout.readline()
        path=self.root/'runtime/vla/owned_processes.json'
        path.write_text(json.dumps({'boot_id':'another-boot','processes':[process(child.pid)]}))
        self.assertFalse(remaining(self.root).items)


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        for folder in ('fly/runtime','runtime/vla'):(self.root/folder).mkdir(parents=True)
        self.paths=patch.multiple(bench_cycle,BASE=self.root/'fly',ROOT=self.root)
        self.paths.start()
        self.backend=bench_cycle.Backend('test')
        self.backend.command=Mock();self.backend.event=Mock()
        self.backend.listening=Mock(return_value=False)
        self.backend.port_bound=Mock(return_value=False)

    def tearDown(self):
        self.paths.stop();self.temp.cleanup()

    def test_no_ports_still_reaps_leftover_and_checks_supervisor(self):
        group=Mock();group.refresh.return_value=[]
        with patch.object(bench_cycle,'remaining',return_value=group),patch.object(bench_cycle,'save'):
            self.backend.stop_stack()
        group.stop.assert_called_once()
        self.assertEqual(self.backend.command.call_args.args[0][-1],'stop')
        self.backend.event.assert_called_once()

    def test_failed_ground_guard_does_not_signal_or_report_success(self):
        self.backend.listening.return_value=True
        self.backend.command.side_effect=RuntimeError('airborne')
        with patch.object(bench_cycle,'remaining') as recover:
            with self.assertRaisesRegex(RuntimeError,'airborne'):self.backend.stop_stack()
        recover.assert_not_called();self.backend.event.assert_not_called()

    def test_survivor_does_not_report_shutdown_complete(self):
        group=Mock();group.stop.side_effect=RuntimeError('survivor')
        with patch.object(bench_cycle,'remaining',return_value=group):
            with self.assertRaisesRegex(RuntimeError,'survivor'):self.backend.stop_stack()
        self.backend.event.assert_not_called()

    def test_late_survivor_is_detected_even_when_all_ports_are_free(self):
        group=Mock();group.refresh.return_value=[{'pid':123}]
        with patch.object(bench_cycle,'remaining',return_value=group),patch.object(bench_cycle,'save'):
            with self.assertRaisesRegex(RuntimeError,'123'):self.backend.stop_stack()
        self.backend.event.assert_not_called()

    def test_live_supervisor_without_master_refusal_preserves_nodes(self):
        self.backend.command.side_effect=RuntimeError('master missing')
        with patch.object(bench_cycle,'remaining') as recover:
            with self.assertRaisesRegex(RuntimeError,'master missing'):self.backend.stop_stack()
        recover.assert_not_called();self.backend.event.assert_not_called()


class SupervisorTests(unittest.TestCase):
    def test_master_startup_failure_runs_cleanup(self):
        spec=importlib.util.spec_from_file_location('cleanup_supervisor',Path(__file__).resolve().with_name('vla_stack.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            (root/'vla_config.json').write_text(json.dumps({'fcu_url':'dummy','inference_url':'dummy','instruction':'test'}))
            group=Mock();group.refresh.return_value=[]
            with patch.multiple(module,BASE=root,STATE=root),patch.object(module,'master',return_value=False),\
                 patch.object(module,'remaining',return_value=group),patch.object(module,'Processes',return_value=group),\
                 patch.object(module,'save'),patch.object(module.signal,'signal'),\
                 patch.object(module.subprocess,'Popen',side_effect=OSError('simulated startup failure')):
                with self.assertRaisesRegex(OSError,'simulated startup failure'):module.daemon()
            group.stop.assert_called_once()
            self.assertTrue(json.loads((root/'status.json').read_text())['stopped'])


if __name__=='__main__':unittest.main(verbosity=2)
