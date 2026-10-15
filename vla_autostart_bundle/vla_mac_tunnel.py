#!/usr/bin/env python3
"""Mac companion: passwordless restricted SSH forwards + 4090 inference guard."""
import argparse,fcntl,json,os,signal,subprocess,time,socket
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--identity',required=True);p.add_argument('--known-hosts',required=True);p.add_argument('--state',required=True);a=p.parse_args()
state=Path(a.state);state.mkdir(parents=True,exist_ok=True)
lock=open(state/'lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Tunnel supervisor already running')
(state/'pid').write_text(str(os.getpid()))
base=['ssh','-T','-i',a.identity,'-o','IdentitiesOnly=yes','-o','BatchMode=yes','-o','UserKnownHostsFile="'+a.known_hosts+'"','-o','ConnectTimeout=8','-o','ServerAliveInterval=10','-o','ServerAliveCountMax=3','-o','ExitOnForwardFailure=yes']
commands={'gpu':base+['-N','-p','7027','-L','127.0.0.1:15017:127.0.0.1:5007','howard@39.105.30.146'],'uav':base+['-N','-R','127.0.0.1:15017:127.0.0.1:15017','nvidia@192.168.1.108']}
children={};started={};stopping=False;next_guard=0
logs={k:open(state/(k+'.log'),'a',buffering=1) for k in commands}
def stop(*_):
 global stopping
 stopping=True
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
try:
 while not stopping:
  for name,cmd in commands.items():
   child=children.get(name)
   if child is None or child.poll() is not None:
    children[name]=subprocess.Popen(cmd,stdout=logs[name],stderr=subprocess.STDOUT)
    started[name]=time.monotonic()
  if time.monotonic()>=next_guard:
   try:
    r=subprocess.run(base+['-p','7027','howard@39.105.30.146','ensure'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15,text=True)
    (state/'inference_guard.txt').write_text(r.stdout)
   except Exception as e:(state/'inference_guard.txt').write_text(str(e))
   next_guard=time.monotonic()+30
  try:
   with socket.create_connection(('127.0.0.1',15017),timeout=1):local_ready=True
  except OSError:local_ready=False
  (state/'status.json').write_text(json.dumps({'pid':os.getpid(),'updated_unix':time.time(),'forwards':{k:{'pid':v.pid,'running':v.poll() is None and time.monotonic()-started[k]>2,'process_alive':v.poll() is None} for k,v in children.items()},'local_port':15017,'local_forward_listening':local_ready,'nx_remote_port':15017},indent=2))
  for _ in range(10):
   if stopping:break
   time.sleep(.5)
finally:
 for child in children.values():
  if child.poll() is None:child.terminate()
 for child in children.values():
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:child.kill()
 (state/'pid').unlink(missing_ok=True)
