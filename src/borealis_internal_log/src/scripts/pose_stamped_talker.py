#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def talker():
    pub = rospy.Publisher('PoseStampedchatter', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Pose Stamped running!")
    while not rospy.is_shutdown():
        tmp = PoseStamped()
        tmp.pose.position.x = 1
        tmp.pose.position.y = 1
        tmp.pose.position.z = 1
        tmp.pose.orientation.w = 1

        pub.publish(tmp)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass