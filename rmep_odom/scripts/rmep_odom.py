#!/usr/bin/env python3
import pandas as pd
import numpy as np
import math
import rospy
import rospkg
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler


if __name__ == '__main__':
    rp = rospkg.RosPack()
    pkg_path = rp.get_path("rmep_odom")
    file_path = pkg_path + "/data/data.xlsx"
    data = pd.read_excel(file_path)

    # init
    rospy.init_node('turtlesim_odom')
    odom_pub = rospy.Publisher("rmep/odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("rmep/path", Path, queue_size=10)
    
    x = 0.0
    y = 0.0
    th = 0.0

    r = 0.05
    a = 0.10
    b = 0.10

    odom = Odometry()
    path = Path()
    pose_stamped = PoseStamped()
    # rate
    rate = rospy.Rate(100.0)
    n = 2
    while n < data.shape[0] and not rospy.is_shutdown():
        t = n * 0.01

        w0 = data.encoder1[n]
        w1 = data.encoder2[n]
        w2 = data.encoder3[n]
        w3 = data.encoder4[n]

        vx = (-w0 + w1 + w2 - w3) * r / 4
        vy = (-w0 - w1 + w2 + w3) * r / 4
        vth = (-w0 - w1 - w2 - w3) * r / (4 * (a + b)) 

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        dt = 0.01
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        quat = quaternion_from_euler(0, 0, th)
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.child_frame_id = "base_link"
        pose_stamped.pose = odom.pose.pose
        pose_stamped.header.frame_id = "odom"
        pose_stamped.header.stamp = odom.header.stamp
        path.poses.append(pose_stamped)
        rospy.loginfo("call back")
        odom_pub.publish(odom)
        
        if n % 10 == 0:
            path.header.frame_id = "odom"
            path.header.stamp = rospy.Time.now()
            path_pub.publish(path)

        n = n + 1
        rate.sleep()


    rospy.spin()