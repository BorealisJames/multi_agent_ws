#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


if __name__ == '__main__':
    print("Converting from linux time to sane time")