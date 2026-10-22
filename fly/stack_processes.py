"""Track project processes across roslaunch exits and separate ROS sessions."""
import json
import os
import signal
import time
from pathlib import Path


def process(pid):
    try:
        path=Path('/proc')/str(pid)
        fields=(path/'stat').read_text().rsplit(')',1)[1].split()
        if fields[0] in ('Z','X'):return None
        return {'pid':int(pid),'ppid':int(fields[1]),'start':int(fields[19])}
    except (OSError,ValueError,IndexError):return None


def alive(item):
    try:
        if item['pid']<=1 or item['pid']==os.getpid():return False
        current=process(item['pid'])
        return current is not None and current['start']==item['start']
    except (KeyError,TypeError):return False


class Processes:
    def __init__(self,items=()):
        self.items={p['pid']:p for p in items if alive(p)}

    def add(self,pid):
        item=process(pid)
        if item:self.items[pid]=item
        self.refresh()

    def refresh(self):
        table={}
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                item=process(int(path.name))
                if item:table[item['pid']]=item
        self.items={pid:p for pid,p in self.items.items()
                    if pid in table and table[pid]['start']==p['start']}
        # ROS nodes may create their own sessions/process groups. Follow PPIDs.
        while True:
            children={pid:p for pid,p in table.items()
                      if p['ppid'] in self.items and pid not in self.items}
            if not children:break
            self.items.update(children)
        return list(self.items.values())

    def stop(self,timeouts=(8.,3.,2.)):
        for sig,timeout in zip((signal.SIGINT,signal.SIGTERM,signal.SIGKILL),timeouts):
            sent=set()
            deadline=time.monotonic()+timeout
            while self.refresh():
                for item in list(self.items.values()):
                    identity=(item['pid'],item['start'])
                    if identity not in sent and alive(item):
                        try:os.kill(item['pid'],sig)
                        except ProcessLookupError:pass
                        sent.add(identity)
                if time.monotonic()>=deadline:break
                time.sleep(.05)
            if not self.items:return
        raise RuntimeError('Project processes did not exit: '+str(sorted(self.items)))


def save(path,groups):
    items={p['pid']:p for group in groups for p in group.refresh()}
    data={'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
          'processes':list(items.values())}
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data));tmp.replace(path)


def remaining(root):
    """Recover tracked descendants, plus narrowly matched legacy project nodes."""
    items=[]
    path=root/'runtime/vla/owned_processes.json'
    try:
        data=json.loads(path.read_text())
        if data['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            items=data['processes']
    except (OSError,ValueError,KeyError,TypeError):pass
    result=Processes(items)
    binaries=[root/'odin_ws/devel/lib/odin_ros_driver',root/'agile_ws/devel/lib/uav_control_fsm']
    scripts={root/'fly/relay.py',root/'fly/fly_cli.py',root/'vla_direct_bridge.py',
             root/'vla_monitor.py',root/'vla_report_publisher.py'}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
        try:
            if proc.stat().st_uid!=os.getuid():continue
            exe=(proc/'exe').resolve()
            argv=[os.fsdecode(s) for s in (proc/'cmdline').read_bytes().split(b'\0') if s]
            match=any(folder in exe.parents for folder in binaries)
            if not match and 'python' in exe.name:
                index=1
                while index<len(argv) and argv[index] in ('-u','-B'):index+=1
                if index<len(argv) and not argv[index].startswith('-'):
                    script=Path(argv[index])
                    if not script.is_absolute():script=(proc/'cwd').resolve()/script
                    match=script.resolve() in scripts
            if match:result.add(int(proc.name))
        except (OSError,ValueError):continue
    result.refresh()
    return result
