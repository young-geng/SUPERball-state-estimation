# Python imports
import numpy as np
import copy

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

# ROS node
rospy.init_node('ntrt_to_gps_state')
ros_hz = 10.0
ros_freq = 1.0/ros_hz
r = rospy.Rate(ros_hz)

# Global Data
node_pos = np.zeros((12,3))
node_vec = np.zeros((12,3))
node_accel = np.zeros((12,3))
noise_on = 1

# Noise Parameters
vec_mu = 0
vec_std = 1.00

# Do not set to > 1
vec_percent_success = 0.95
# Percent Dropped
vpd = vec_percent_success / 0.684

# Bar Velocity and Acceleration Prev variables
prev_node_pos = np.zeros_like(node_pos)
prev_node_vel = np.zeros_like(node_pos)

# Accel Calculation
def accel_calc(tmp_node_pos):
    global prev_node_pos
    global prev_node_vel 

    # Bar Vectors
    node_vec[1:12:2] = tmp_node_pos[1:12:2] - tmp_node_pos[0:11:2]   
    norm = np.linalg.norm(node_vec[1:12:2], axis=1)
    node_vec[1:12:2] = node_vec[1:12:2.]/np.array([norm, norm, norm]).T
    node_vec[0:11:2] = node_vec[1:12:2]
    
    # Bar Accels
    node_vel = (tmp_node_pos - prev_node_pos) / ros_freq
    node_accel = (node_vel - prev_node_vel) / ros_freq
    for i in range(12):
        node_accel[i][1] += np.dot(node_vec[i],np.array([0,0,(2*(i%2)-1)*9.81]))
    prev_node_pos = copy.copy(tmp_node_pos)
    prev_node_vel = copy.copy(node_vel)

    node_accel_noise = node_accel.reshape(1,-1) + np.random.normal(vec_mu, vec_std, node_vec.size)
    node_accel_noise = node_accel.reshape(12,3)

    for i in range(12):
        accel_msg.header.frame_id = 'node %d'%node_ids[i]
        accel_msg.header.stamp = rospy.get_rostime()
        #accel_msg.linear_acceleration.x = node_accel[i][0]
        if(noise_on == 1):
            accel_msg.linear_acceleration.y = node_accel_noise[i][1]
        else:
            accel_msg.linear_acceleration.y = node_accel[i][1]
        #accel_msg.linear_acceleration.z = node_accel[i][2]
        accel_msg.linear_acceleration_covariance[0] = -1
        pub_accel[i].publish(accel_msg)

# Subscriber callback function
def state_cb(msg):
    pubMsg = SUPERballStateArray()
    #motor_pos = msg.data[-12:]
    #tmp_node_pos = msg.data[:-12]
    #print motor_pos
    for idx,m in enumerate(msg.states):
        state = SUPERballState()

        state.motor_pos1.data = m.motor_pos1.data #0.95 - np.abs((0.009)*motor_pos[2*idx])# + (0.009*7.5)
        state.motor_pos2.data = m.motor_pos2.data #0.95 - np.abs((0.009)*motor_pos[2*idx + 1])# + (0.009*7.5)

        node_pos[2*idx] = np.array([m.pos1.x,m.pos1.y,m.pos1.z])
        node_pos[2*idx+1] = np.array([m.pos2.x,m.pos2.y,m.pos2.z])

        pubMsg.states.append(state)
    accel_calc(node_pos)
    pub.publish(pubMsg)


imu_topics = ["/bbb2/0x1_imu_data","/bbb2/0x71_imu_data","/bbb4/0x1_imu_data","/bbb4/0x71_imu_data","/bbb6/0x1_imu_data","/bbb6/0x71_imu_data","/bbb8/0x1_imu_data","/bbb8/0x71_imu_data","/bbb10/0x1_imu_data","/bbb10/0x71_imu_data","/bbb12/0x1_imu_data","/bbb12/0x71_imu_data"]
node_ids = [2,1,4,3,6,5,8,7,10,9,12,11]
pub_accel = []
for m in range(12):
    pub_accel.append(rospy.Publisher(imu_topics[m], Imu, queue_size=1))
accel_msg = Imu()

# Subscriber init
sub = rospy.Subscriber('/superball/state_matlab', SUPERballStateArray, state_cb)

# Publisher init
pub = rospy.Publisher('/superball/state', SUPERballStateArray, queue_size=1)

while(not rospy.is_shutdown()):
    r.sleep()
