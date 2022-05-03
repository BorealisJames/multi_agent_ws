#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

child_id = "child"
parent_id = "world"
translation = [0,0,0]
rotation= [0,0,0,1]

if __name__ == '__main__':
    rospy.init_node('tf2_static_publisher')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent_id
    static_transformStamped.child_frame_id = "child"

    static_transformStamped.transform.translation.x = translation[0]
    static_transformStamped.transform.translation.y = translation[1]
    static_transformStamped.transform.translation.z = translation[2]

    static_transformStamped.transform.rotation.x = rotation[0]
    static_transformStamped.transform.rotation.y = rotation[1]
    static_transformStamped.transform.rotation.z = rotation[2]
    static_transformStamped.transform.rotation.w = rotation[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
