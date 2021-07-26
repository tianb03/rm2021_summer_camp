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
    file_path = pkg_path + "/data/ep_enc.xlsx"
    data = pd.read_excel(file_path)

    # init
    rospy.init_node('turtlesim_odom')
    odom_pub = rospy.Publisher("rmep/odom", Odometry, queue_size=10)
    path_pub = rospy.Publisher("rmep/path", Path, queue_size=10)
    pose_pub = rospy.Publisher("rmep/pose", PoseStamped, queue_size=10)
    
    x = 0.0
    y = 0.0
    th = 0.0

    r = 0.05
    a = 0.10
    b = 0.10

    path = Path()

    # rate
    rate = rospy.Rate(100.0)
    n = 1
    while n < data.shape[0] and not rospy.is_shutdown():
        dt = 0.01

        w0 = ( data.encoder1[n] - data.encoder1[n-1] )
        if w0 > math.pi:
            w0 = w0 - math.pi * 2
        elif w0 < -math.pi:
            w0 = w0 + math.pi * 2
        w1 = data.encoder2[n] - data.encoder2[n-1]
        if w1 > math.pi:
            w1 = w1 - math.pi * 2
        elif w1 < -math.pi:
            w1 = w1 + math.pi * 2
        w2 = data.encoder3[n] - data.encoder3[n-1]
        if w2 > math.pi:
            w2 = w2 - math.pi * 2
        elif w2 < -math.pi:
            w2 = w2 + math.pi * 2
        w3 = data.encoder4[n] - data.encoder4[n-1]
        if w3 > math.pi:
            w3 = w3 - math.pi * 2
        elif w3 < -math.pi:
            w3 = w3 + math.pi * 2

        # vx = (-w0 + w1 + w2 - w3) * r / 4 / dt
        # vy = (-w0 - w1 + w2 + w3) * r / 4 / dt
        # vth = (-w0 - w1 - w2 - w3) * r / (4 * (a + b)) /dt 

        vx = (w0 + w1 + w2 + w3) * r / 4 / dt
        vy = (-w0 + w1 + w2 - w3) * r / 4 / dt
        vth = (-w0 + w1 - w2 + w3) * r / (4 * (a + b)) /dt 
        odom = Odometry() # can only be defined here inside the loop
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        quat = quaternion_from_euler(0, 0, th)
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.header.seq = n

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.child_frame_id = "base_link"

        pose_stamped = PoseStamped()
        pose_stamped.pose = odom.pose.pose
        pose_stamped.header = odom.header

        path.header = odom.header
        path.poses.append(pose_stamped)

        rospy.loginfo(len(path.poses))
        odom_pub.publish(odom)
        # pose_pub.publish(path.poses)
        if n % 10 == 0: 
            path_pub.publish(path)

        n = n + 1
        rate.sleep()

    rospy.spin()