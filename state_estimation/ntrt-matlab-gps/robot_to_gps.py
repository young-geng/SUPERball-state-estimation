# Python imports
import numpy as np

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray

# ROS node
rospy.init_node('robot_to_gps_state')

# Global Data
prev_data = np.zeros((12,1))

# Subscriber callback function
def state_cb(msg):
    global prev_data
    pubMsg = SUPERballStateArray()
    motor_pos = np.array(msg.data[-12:])
    #node_pos = msg.data[:-12]
    for idx in xrange(6):
        state = SUPERballState()

        motor_pos[np.isnan(motor_pos)] = prev_data[np.isnan(motor_pos)]

        #state.motor_pos1.data = 1 - (0.009)*motor_pos[2*idx]# + (0.009*7.5)
        #state.motor_pos2.data = 1 - (0.009)*motor_pos[2*idx + 1]# + (0.009*7.5)
        state.motor_pos1.data = 0.95 - np.abs((0.009)*motor_pos[2*idx])# + (0.009*7.5)
        state.motor_pos2.data = 0.95 - np.abs((0.009)*motor_pos[2*idx + 1])# + (0.009*7.5)

        pubMsg.states.append(state)
    print motor_pos
    pub.publish(pubMsg)
    prev_data = motor_pos 

# Subscriber init
sub = rospy.Subscriber('/ranging_data_matlab', Float32MultiArray, state_cb)

# Publisher init
pub = rospy.Publisher('/superball/state_sim', SUPERballStateArray, queue_size=1)

r = rospy.Rate(100)
while(not rospy.is_shutdown()):
    r.sleep()
