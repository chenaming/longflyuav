#!/usr/bin/env python3
"""Publish only reports belonging to the currently active bridge run."""
import json,time
from pathlib import Path
import rospy
from std_msgs.msg import String
rospy.init_node('vla_report_evidence')
base=Path('/home/nvidia/agile/runtime/vla')
pubs={kind:rospy.Publisher(topic,String,queue_size=1,latch=True) for kind,topic in [('diagnostic','/vla/bench_report'),('model','/vla/model_bench_report'),('motor','/vla/model_motor_report')]}
last={}
while not rospy.is_shutdown():
 for kind,pub in pubs.items():
  try:
   text=(base/(kind+'_report.json')).read_text();data=json.loads(text)
   if data.get('run_id')==rospy.get_param('/vla_direct_bridge/run_id',''):
    pub.publish(String(data=text));last[kind]=text
  except (OSError,ValueError):pass
 time.sleep(2.)
