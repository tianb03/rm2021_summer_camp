#!/usr/bin/env python3
import time
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from rospy.core import rospyinfo
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler


x = 0.0
y = 0.0
th = 0.0

def cmd_vel_cb(msg):
    global x
    global y
    global th
    current_time = rospy.Time.now()
    odom = Odometry()
    odom.twist.twist = msg
    vx = msg.linear.x
    vy = msg.linear.y
    vth = msg.angular.z

    dt = 0.1
    delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
    delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    quat = quaternion_from_euler(0, 0, th)
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation.x = quat[0]
    odom.pose.pose.orientation.y = quat[1]
    odom.pose.pose.orientation.z = quat[2]
    odom.pose.pose.orientation.w = quat[3]
    odom.child_frame_id = "base_link"

    rospy.loginfo("call back")
    pub.publish(odom)

if __name__ == '__main__':

    # init
    rospy.init_node('turtlesim_odom')
    sub = rospy.Subscriber("turtle1/cmd_vel", Twist, cmd_vel_cb)
    pub = rospy.Publisher('turtle1/odometry', Odometry, queue_size=10)
    rospy.spin()