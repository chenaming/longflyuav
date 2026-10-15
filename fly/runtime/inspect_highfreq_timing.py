import rospy,time,json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
rospy.init_node('inspect_highfreq_timing',anonymous=True)
rows={}
def receive(k,m):
 rows.setdefault(k,[]).append((time.monotonic(),m.header.stamp.to_sec(),rospy.Time.now().to_sec()-m.header.stamp.to_sec()))
for k,topic,cls in [('high','/uav1/odin1/odometry_highfreq',Odometry),('normal','/uav1/odin1/odometry',Odometry),('vision','/uav1/mavros/vision_pose/pose',PoseStamped)]:
 rospy.Subscriber(topic,cls,lambda m,k=k:receive(k,m),queue_size=2000)
time.sleep(5)
for k,v in rows.items():
 gaps=[b[1]-a[1] for a,b in zip(v,v[1:])]
 print('TIMING',k,json.dumps({'count':len(v),'hz':(len(v)-1)/(v[-1][0]-v[0][0]),
 'unique':len({r[1] for r in v}),'future_count':sum(r[2]<0 for r in v),
 'age_min':min(r[2] for r in v),'age_max':max(r[2] for r in v),
 'stamp_gap_min':min(gaps),'stamp_gap_max':max(gaps),'backwards':sum(g<0 for g in gaps)}),flush=True)
