#Normal python imports
import numpy as np
import scipy.spatial as ss
import scipy.io

# ROS imports
import roslib; roslib.load_manifest('gps_agent_pkg')
import rospy
from gps_agent_pkg.msg import SUPERballState, SUPERballStateArray
from std_msgs.msg import Float32MultiArray, Float32, UInt16
from superball_msg.msg import TimestampedFloat32

# ROS node
rospy.init_node("manual_motor")

# Global variables
motor_pos = [1.0, #1
             1.0, #2
             1.0, #3
             1.0, #4
             1.0, #5
             1.0, #6
             1.0, #7
             1.0, #8
             1.0, #9
             1.0, #10
             1.0, #11
             1.0] #12

# Setup ROS publishers for motor positions
motor_pubs = []
for i in range(12):
    if i % 2 == 0:
        bbb, board_id, sub_index = i + 2, 0x71, 0x2
    else:
        bbb, board_id, sub_index = i + 1, 0x1, 0x1
    motor_pubs.append(rospy.Publisher('/bbb%d/0x%x_0x2040_0x%x' % (bbb, board_id, sub_index), \
                           TimestampedFloat32, queue_size=1))

time_pub = rospy.Publisher('/superball/timestep', UInt16, queue_size=1)

# Testing single motor movements
motor_move = [8,12,4,2,5,9] 
move_amount = 48
count = 1
index = 2
lastIndex = 5
rate = 100 #in ms
move = 0
out = 0
motor_power = 0.7 #Percentage of avaliable motor power

# Set program rate and main loop
r = rospy.Rate(1000/rate)
while(not rospy.is_shutdown()):
    if(count%(6*(1000/rate)) == 0):
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
        msg = TimestampedFloat32()
        msg.header.stamp = rospy.rostime.get_rostime()
        if(idx == (motor_move[index]-1)):
            msg.data = motor_pos[idx] - move
            motor_pubs[idx].publish(msg)
        elif(idx == (motor_move[lastIndex]-1)):
            msg.data = motor_pos[idx] - out
            motor_pubs[idx].publish(msg)
        else:
            msg.data = motor_pos[idx]
            motor_pubs[idx].publish(msg)
    print motor_pubs[motor_move[index]-1].name
    print motor_pubs[motor_move[lastIndex]-1].name
    time_pub.publish(rate)
    r.sleep()
