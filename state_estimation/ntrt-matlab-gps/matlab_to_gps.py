# Python imports
import numpy as np

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray

# ROS node
rospy.init_node('matlab_to_gps_state')

# Global Data

# Subscriber callback function
def state_cb(msg):
    pubMsg = SUPERballStateArray()
    motor_pos = msg.data[-12:]
    node_pos = msg.data[:-12]
    print motor_pos
    for idx in xrange(6):
        state = SUPERballState()

        state.pos1.x = node_pos[6*idx]
        state.pos1.y = node_pos[6*idx + 1]
        state.pos1.z = node_pos[6*idx + 2]
        state.pos2.x = node_pos[6*idx + 3]
        state.pos2.y = node_pos[6*idx + 4]
        state.pos2.z = node_pos[6*idx + 5]

        state.motor_pos1.data = motor_pos[2*idx]
        state.motor_pos2.data = motor_pos[2*idx + 1]

        pubMsg.states.append(state)
    pub.publish(pubMsg)

# Subscriber init
sub = rospy.Subscriber('/matlab_superball_state', Float32MultiArray, state_cb)

# Publisher init
pub = rospy.Publisher('/superball/state', SUPERballStateArray, queue_size=1)

r = rospy.Rate(100)
while(not rospy.is_shutdown()):
    r.sleep()
