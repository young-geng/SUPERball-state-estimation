#Normal python imports
import numpy as np
import scipy.spatial as ss
import scipy.io

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray

# ROS node
rospy.init_node("sim_data_to_matlab")

# Global Variables
node_pos = np.zeros((12,3))
motor_pos = np.zeros((12,1))
all_data = np.zeros((210,1))
noise_on = 0

# Node offset to move the robot around.
node_offset = np.array([2.0, -2.0, 0.0])

node_vec = np.ones((12,3))
# Load base station positions from a mat file and sort them into acending order
base_mat = scipy.io.loadmat('positions.mat')
base_pos_order = np.hstack((base_mat.values()[0],base_mat.values()[2].T))
base_pos = base_pos_order[base_pos_order[:,3].argsort()][:,0:3]
base_pos[:,2] = base_pos[:,2]*-1 + 2.2
#base_pos[6:7] = np.ones((1,3))*np.nan

#base_pos = np.array([[1, 1, 1],
#                    [2, 2, 2],
#                    [2, 2, 2],
#                    [2, 2, 2],
#                    [2, 2, 2],
#                    [2, 2, 2],
#                    [2, 2, 2],
#                    [2, 2, 2]])

# Simulate Rods being on or not
rod_enable = [1, #1-2
              1, #3-4
              1, #5-6
              1, #7-8
              1, #9-10
              1] #11-12

rod_enable_ranging = [1,
                      1, #2
                      1, #3
                      1, #4
                      1, #5
                      1, #6
                      1, #7
                      1, #8
                      1, #9
                      1, #10
                      1, #11
                      1, #12
                      1, #13
                      1, #14
                      1, #15
                      1, #16
                      1, #17
                      1, #18
                      1, #19
                      1] #20

# Subscriber callback function
def state_cb(msg):
    for idx,m in enumerate(msg.states):
        if(rod_enable[idx]):
            node_pos[2*idx] = [m.pos1.x, m.pos1.y, m.pos1.z] + node_offset
            node_pos[2*idx+1] = [m.pos2.x, m.pos2.y, m.pos2.z] + node_offset

            motor_pos[2*idx] = (1 - m.motor_pos1.data) / 0.08257
            motor_pos[2*idx+1] = (1 - m.motor_pos2.data) / 0.08257
        else:
            node_pos[2*idx] = [np.nan, np.nan, np.nan]
            node_pos[2*idx+1] = [np.nan, np.nan, np.nan]

            motor_pos[2*idx] = np.nan 
            motor_pos[2*idx+1] = np.nan

# Subscriber for NTRT/GPS message
sub = rospy.Subscriber('/superball/state_matlab', SUPERballStateArray, state_cb)

pub = rospy.Publisher('/ranging_data_matlab_sim', Float32MultiArray, queue_size=1)
pub_nodes = rospy.Publisher('/node_positions', Float32MultiArray, queue_size = 1)
r = rospy.Rate(10)
msg = Float32MultiArray()
temp_node_pos = np.zeros((12,3))
temp_base_pos = np.zeros((8,3))

# Noise Parameters
ranging_mu = 0
ranging_std = 0.2
vec_mu = 0
vec_std = 0.2

# Do not set to > 1
ranging_percent_success = 0.1
vec_percent_success = 0.90

rpd = ranging_percent_success / 0.684 
vpd = vec_percent_success / 0.684

while(not rospy.is_shutdown()):
    #process data here
    #Length Measures
    internal_dist = (np.ones((66,1)))*np.nan
    temp_node_pos = node_pos[:].copy()
    temp_base_pos = base_pos[:].copy()
    for i in xrange(np.size(rod_enable_ranging)):
        if(not rod_enable_ranging[i]):
            if(i > 11):
                temp_base_pos[(i-12)] = np.array([np.nan, np.nan, np.nan])
            else:
                temp_node_pos[i] = np.array([np.nan, np.nan, np.nan])
    external_dist = ss.distance.cdist(temp_node_pos,temp_base_pos,'euclidean')

    # Bar Vectors
    node_vec[1:12:2] = node_pos[1:12:2] - node_pos[0:11:2]   
    norm = np.linalg.norm(node_vec[1:12:2], axis=1)
    node_vec[1:12:2] = node_vec[1:12:2.]/np.array([norm, norm, norm]).T
    node_vec[0:11:2] = node_vec[1:12:2]
    
    if(noise_on):
        # Add noise
        external_dist_noise = external_dist.reshape(1,-1) + np.random.normal(ranging_mu, ranging_std, external_dist.size)
        node_vec_noise = node_vec.reshape(1,-1) + np.random.normal(vec_mu, vec_std, node_vec.size)

        # generate and remove dropped messages
        ranging_drop = np.random.normal(0, 1, external_dist.size)
        vec_drop = np.random.normal(0, 1, node_vec.size)
        for idx in xrange(external_dist_noise.size):
            if(np.abs(ranging_drop[idx]) > rpd):
                external_dist_noise[0][idx] = np.nan
        for idx in xrange(node_vec_noise.size):
            if(np.abs(vec_drop[idx]) > vpd):
                node_vec_noise[0][idx] = np.nan

        # Stack all measurements together
        all_data = np.vstack((internal_dist,external_dist_noise.reshape(-1,1),node_vec_noise.reshape(-1,1),motor_pos.reshape(-1,1)))
    else:
        # Stack all measurements together
        all_data = np.vstack((internal_dist,external_dist.reshape(-1,1),node_vec.reshape(-1,1),motor_pos.reshape(-1,1)))

    # Send the data out over ROS
    msg.data = (all_data.T[0]).tolist()
    #all_data[:] = np.nan
    #node_pos[:] = np.nan
    pub.publish(msg)

    msg.data = node_pos.ravel().tolist()
    pub_nodes.publish(msg)

    print 'Node:'
    print node_pos
    #print motor_pos.reshape(-1,1) 
    print 'Base:'
    print base_pos
    #print 'Dist Vector'
    #print external_dist.reshape(-1,1)    
    #print external_dist
    r.sleep()
    

