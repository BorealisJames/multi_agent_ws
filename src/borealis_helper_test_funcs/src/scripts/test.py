import os
import rospy
import math
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String
from borealis_msgs.msg import Borealis_HRI_Output

if __name__ == '__main__':
    what = Borealis_HRI_Output()
    what2 = PoseArray()
    what3 = PoseStamped()

    what3 = what.uav_pose_array.poses[0]
    