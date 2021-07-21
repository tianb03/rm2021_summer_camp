#!/usr/bin/env python3

import rospy

import math
import tf
from tf import transformations
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener2 = tf.TransformListener()
    listener3 = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner2 = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner2(4, 4, 0, 'turtle2')

    turtle2_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rospy.wait_for_service('spawn')
    spawner3 = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner3(4, 7, 0, 'turtle3')

    turtle3_vel = rospy.Publisher('turtle3/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans2, rot2) = listener2.lookupTransform('/turtle2', '/turtle2_target', rospy.Time())
            (trans3, rot3) = listener3.lookupTransform('/turtle3', '/turtle3_target', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # angular2 = 4 * math.atan2(trans2[1], trans2[0])
        angular2 = 5 * transformations.euler_from_quaternion(rot2)[2]
        # linear2 = 0.5 * math.sqrt(trans2[0] ** 2 + trans2[1] ** 2)
        linear_x2 = 5 * trans2[0]
        linear_y2 = 5 * trans2[1]
        msg2 = geometry_msgs.msg.Twist()
        msg2.linear.x = linear_x2
        msg2.linear.y = linear_y2
        msg2.angular.z = angular2
        turtle2_vel.publish(msg2)

        angular3 = 4 * math.atan2(trans3[1], trans3[0])
        linear3 = 0.5 * math.sqrt(trans3[0] ** 2 + trans3[1] ** 2)
        msg3 = geometry_msgs.msg.Twist()
        msg3.linear.x = linear3
        msg3.angular.z = angular3
        turtle3_vel.publish(msg3)

        rate.sleep()
