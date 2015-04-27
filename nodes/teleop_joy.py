#!/usr/bin/env python
## @package teleop_joy A node for controlling the P3DX with an XBox controller

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Joy
import numpy as np

def quat2yaw(q):
    return np.arctan2(2*(q.y*q.z + q.w*q.x), 1 - 2*(q.z**2 + q.w**2))

def joyCallback(msg):
    global cmd_vel_pub

    global linear_axis
    global linear_scale
    global rotation_axis
    global rotation_scale
    global yaw

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = msg.axes[linear_axis] * linear_scale
    cmd_vel_msg.angular.z = msg.axes[rotation_axis] * rotation_scale
    cmd_vel_msg.angular.y = np.inf
    cmd_vel_pub.publish(cmd_vel_msg)



if __name__ == '__main__':
    rospy.init_node('teleop_joy')
    global cmd_vel_pub

    global linear_axis
    global linear_scale
    global rotation_axis
    global rotation_scale
    global yaw

    linear_axis = rospy.get_param('linear_axis' , 1)
    linear_scale = rospy.get_param('linear_scale' , 5)

    rotation_axis = rospy.get_param('rotation_axis' , 3)
    rotation_scale = rospy.get_param('rotation_scale', 1)

    cmd_vel_pub = rospy.Publisher("/asv/cmd_vel", Twist, queue_size=1)

    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.spin()
