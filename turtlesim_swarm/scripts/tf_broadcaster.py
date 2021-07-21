#!/usr/bin/env python3

import rospy

import tf
import turtlesim.msg

br = tf.TransformBroadcaster()
br21 = tf.TransformBroadcaster()
br31 = tf.TransformBroadcaster()

def handle_turtle_pose(msg, turtlename):

    # a static broadcaster to pub the target frame
    # broadcaster = tf2_ros.StaticTransformBroadcaster()

    # static_transformStamped = TransformStamped()
    # static_transformStamped.header.stamp = rospy.Time.now()
    # static_transformStamped.header.frame_id = tag_name
    # target_frame = tag_name + "_target"
    # static_transformStamped.child_frame_id = target_frame

    # static_transformStamped.transform.translation.y -= track_distance*np.tan(np.deg2rad(15))
    # static_transformStamped.transform.translation.z += track_distance 

    # quat = quaternion_from_euler(0, np.deg2rad(90), np.deg2rad(-90))
    # static_transformStamped.transform.rotation.x = quat[0]
    # static_transformStamped.transform.rotation.y = quat[1]
    # static_transformStamped.transform.rotation.z = quat[2]
    # static_transformStamped.transform.rotation.w = quat[3]
    # broadcaster.sendTransform(static_transformStamped)

    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     turtlename,
                     "world")
    if turtlename == "turtle1":
        br21.sendTransform((-1, 1, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "turtle2_target",
                        "turtle1")
        br31.sendTransform((-1, -1, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "turtle3_target",
                        "turtle1")



if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    # turtlename = "turtle1"
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()

