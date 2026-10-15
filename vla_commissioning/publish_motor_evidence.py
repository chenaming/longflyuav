import json
from pathlib import Path
import rospy
from std_msgs.msg import String
rospy.init_node('vla_motor_evidence')
p=rospy.Publisher('/vla/model_motor_report',String,queue_size=1,latch=True)
s=Path('/home/nvidia/.local/state/vla-commissioning/motor_model_test/motor_report_verified.json').read_text()
p.publish(String(data=s))
rospy.spin()  # Evidence only; no motor/command publishers or services.
