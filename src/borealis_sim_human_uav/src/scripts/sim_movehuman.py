#!/usr/bin/env python

"""
The UAV is Sam
Mave bot move and follow Sam's movement in ROSbag.
Make it 1.7th m tall to have a better gauge.
It waits for a msg from a topic, if there is no msg it will hover at 0,0,2.
Once rosbag.launch is launched, the human UAV will go to the positions set by the rosbag
"""

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

global pose_stamped
pose_stamped = PoseStamped()

def follow_pose_cb(data):
    pose_stamped.pose.position.x = data.pose.pose.position.x
    pose_stamped.pose.position.y = data.pose.pose.position.y
    pose_stamped.pose.position.z = 2

    pose_stamped.pose.orientation.x = data.pose.pose.orientation.x
    pose_stamped.pose.orientation.y = data.pose.pose.orientation.y
    pose_stamped.pose.orientation.z = data.pose.pose.orientation.z
    pose_stamped.pose.orientation.w = data.pose.pose.orientation.w


def sim_state_cb(msg):
    pass

if __name__ == "__main__":

    pose_topic = "/uav_all/follow_me_target_pose"
    publisher_topic = "/human/mavros/setpoint_position/local"

    simulation_topic = "/sim_state"
    simulation_state = rospy.get_param(simulation_topic, 'waiting')

    rospy.init_node('borealis_sam', anonymous=True)

    sub = rospy.Subscriber(pose_topic, Odometry, follow_pose_cb)
    pub = rospy.Publisher(publisher_topic, PoseStamped, queue_size=10)
    pub_sim = rospy.Subscriber(simulation_topic, String, sim_state_cb)
    rate = rospy.Rate(20)

    # While waiting for topic call back, make it hover at 0,0,2
    while simulation_state == 'waiting':
        simulation_state = rospy.get_param(simulation_topic, 'waiting')
        try:
            for i in range (40):
                init_pose = PoseStamped()
                init_pose.pose.position.x = 0
                init_pose.pose.position.y = 0
                init_pose.pose.position.z = 2
                pub.publish(init_pose)
                rate.sleep()
        except rospy.exceptions.ROSException:
            break

    while not rospy.is_shutdown():
        pub.publish(pose_stamped)
        rate.sleep()
    
    rospy.loginfo("Human move begin")
    rospy.spin()
