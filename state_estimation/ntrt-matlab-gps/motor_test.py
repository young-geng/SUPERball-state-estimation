#Normal python imports
import numpy as np
import scipy.spatial as ss
import scipy.io
import time
import sys

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray, Float32, UInt16, String


END_CAP_6_DOWN = True



def compute_distance_callback(msg):
        bar_endpt_pos = []
        for i in range(6):
            pos1, pos2 = msg.states[i].pos1, msg.states[i].pos2
            bar_endpt_pos.extend([[pos1.x, pos1.y, pos1.z], [pos2.x, pos2.y, pos2.z]])
        bar_endpt_pos = np.array(bar_endpt_pos)
        distance = np.linalg.norm(
            [np.mean(bar_endpt_pos[:, 0]), np.mean(bar_endpt_pos[:, 1])]
        )
        global distance_traveled
        distance_traveled = distance

# ROS node
rospy.init_node("manual_motor")

distance_traveled = 0

compute_distance_sub = rospy.Subscriber(
    '/superball/state_sim', SUPERballStateArray,
    callback=compute_distance_callback, queue_size=1
)


control_pub = rospy.Publisher('/superball/control', String, queue_size=1)

# Global variables
motor_pos = [0.0, #1
             0.0, #2
             0.0, #3
             0.0, #4
             0.0, #5
             0.0, #6
             0.0, #7
             0.0, #8
             0.0, #9
             0.0, #10
             0.0, #11
             0.0] #12

# Setup ROS publishers for motor positions
motor_pubs = []
for i in range(12):
    if i % 2 == 0:
        bbb, board_id, sub_index = i + 2, 0x71, 0x2
    else:
        bbb, board_id, sub_index = i + 1, 0x1, 0x1
    motor_pubs.append(rospy.Publisher('/bbb%d/0x%x_0x2040_0x%x' % (bbb, board_id, sub_index), \
                           Float32, queue_size=1))

time_pub = rospy.Publisher('/superball/timestep', UInt16, queue_size=1)



# Testing single motor movements
motor_move = [8,12,4,2,5,9]
move_amount = 60
count = 1
index = 2
lastIndex = 5
rate = 100 #in ms
move = 0
out = 0
motor_power = 0.7 #Percentage of avaliable motor power

# Set program rate and main loop
r = rospy.Rate(1000/rate)

time.sleep(0.5)
rospy.set_param('/bottom_face', 1)
control_pub.publish(String('reset'))
for i in xrange(12):
    motor_pubs[i].publish(0)

time.sleep(0.5)
for _ in xrange(30):
    time_pub.publish(rate)
    r.sleep()


# while(not rospy.is_shutdown()):
for time_step in xrange(600):
    if(count%(4*(1000/rate)) == 0):
        lastIndex = index
        index = index + 1
        out = move
        move = 0
    if(index > 5):
        index = 0
    count = count + 1

    # Controls motor contraction velocity (pulling in)
    if(move < move_amount):
         move = move + ((26 * motor_power) / (1000/rate))
    else:
        move = move_amount

    # Controls motor relax velocity (letting out)
    if(out <= 0):
        out = 0
    else:
        out = out - ((26 * motor_power) / (1000/rate))

    for idx in range(12):
        if END_CAP_6_DOWN:
            if idx == 6:
                continue
        if(idx == (motor_move[index]-1)):
            motor_pubs[idx].publish(motor_pos[idx] - move)
        elif(idx == (motor_move[lastIndex]-1)):
            motor_pubs[idx].publish(motor_pos[idx] - out)
        else:
            motor_pubs[idx].publish(motor_pos[idx])
    # print motor_pubs[motor_move[index]-1].name
    # print motor_pubs[motor_move[lastIndex]-1].name
    time_pub.publish(rate)
    r.sleep()

sys.stdout.write("{}, ".format(distance_traveled))
