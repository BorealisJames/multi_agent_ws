import rospy
import tf
import numpy as np

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

uwb_human_topic = ''
uwb_uav1_topic = ''
uwb_uav2_topic = ''

uwb_human_pose = ''
uwb_uav1_pose = PoseWithCovarianceStamped()
uwb_uav2_pose = PoseWithCovarianceStamped()

t265_uav1_topic = ''
t265_uav2_topic = ''

t265_uav1_pose = PoseWithCovarianceStamped()
t265_uav2_pose = PoseWithCovarianceStamped()

aloam_uav1_topic = ''
aloam_uav2_topic = ''

aloam_uav1_pose = PoseWithCovarianceStamped()
aloam_uav2_pose = PoseWithCovarianceStamped()

localization_stats_topic = 'localization_stats'
vector_difference_in_localization_stats_topic = 'localization_stats_diff_vector'
magnitude_difference_in_localization_stats_topic = 'localization_stats_diff_magnitude'

if __name__ == '__main__':

    def uwb_human_sub_cb(data):
        global uwb_human_pose
        uwb_human_pose = data
        pass 
    def uwb_uav1_sub_cb(data):
        global uwb_uav1_pose 
        uwb_uav1_pose = data
        pass 
    def uwb_uav2_sub_cb(data):
        global uwb_uav2_pose
        uwb_uav2_pose = data
        pass 

    def t265_uav1_sub_cb(data):
        global t265_uav1_pose 
        t265_uav1_pose = data
        pass 

    def t265_uav2_sub_cb(data):
        global t265_uav2_pose
        t265_uav2_pose = data
        pass 

    def aloam_uav1_sub_cb(data):
        global aloam_uav1_pose 
        aloam_uav1_pose = data
        pass 

    def aloam_uav2_sub_cb(data):
        global aloam_uav2_pose
        aloam_uav2_pose = data
        pass 

    # init stuff 
    rospy.init_node('drift_vaues')
    rate = rospy.Rate(1)
    vec_diff_pub = rospy.Publisher(vector_difference_in_localization_stats_topic, String)

    # subscribers
    uwb_human_sub = rospy.Subscriber(uwb_human_topic,PoseWithCovarianceStamped,uwb_human_sub_cb )
    uwb_uav1_sub = rospy.Subscriber(uwb_uav1_topic,PoseWithCovarianceStamped,uwb_uav1_sub_cb )
    uwb_uav2_sub = rospy.Subscriber(uwb_uav2_topic,PoseWithCovarianceStamped,uwb_uav2_sub_cb )

    t265_uav1_sub = rospy.Subscriber(t265_uav1_topic,PoseWithCovarianceStamped, t265_uav1_sub_cb)
    t265_uav2_sub = rospy.Subscriber(t265_uav2_topic,PoseWithCovarianceStamped, t265_uav2_sub_cb)

    aloam_uav1_sub = rospy.Subscriber(aloam_uav1_topic,PoseWithCovarianceStamped, aloam_uav1_sub_cb)
    aloam_uav2_sub = rospy.Subscriber(aloam_uav2_topic,PoseWithCovarianceStamped, aloam_uav2_sub_cb)
    
    # publishers 
    localzation_val_publisher = rospy.Publisher(localization_stats_topic, String)
    vector_diff_publishers = rospy.Publisher(vector_difference_in_localization_stats_topic, String)
    magnitude_diff_publisher = rospy.Publisher(magnitude_difference_in_localization_stats_topic, String)

    # lock till all of them publisher 

    while not rospy.is_shutdown():
    
        # calculate vector difference

        # prepare vector diff format 
        s_vector = """UWB-UAV1 \n
            Rotation in euler angles\n
            UAV1-UWB-T265-X: {} \n
            UAV1-UWB-T265-Y: {} \n
            UAV1-UWB-T265-Z: {} \n
            UAV1-UWB_T265-X_rot: \n
            UAV1-UWB_T265-Y_rot: \n
            UAV1-UWB_T265-Z_rot: \n

            UAV1-UWB-ALOAM: {}... \n

            UWB-UAV2 \n
            UAV2-UWB-T265: {}\n
            UAV2-UWB-ALOAM: {}\n

            UWB-UAV2 \n
            UAV2-UWB-T265: {}\n
            UAV2-UWB-ALOAM: {}\n
            """.format(0, 0, 0, 0, uwb_t265_magdiff_uav1, uwb_t265_magdiff_uav2)

        # calculate magnitude difference
        def calculate_magnitude_difference(pose1, pose2):
            x_diff = pose1.pose.pose.position.x - pose2.pose.pose.position.x
            y_diff = pose1.pose.pose.position.y - pose2.pose.pose.position.y
            z_diff = pose1.pose.pose.position.z - pose2.pose.pose.position.z

            v = np.array([x_diff, y_diff, z_diff])
            magnitude = np.linalg.norm(v)
            return magnitude

        uwb_t265_magdiff_uav1 = calculate_magnitude_difference(uwb_uav1_pose, uwb_uav1_pose)
        uwb_t265_magdiff_uav2 = calculate_magnitude_difference(uwb_uav2_pose, uwb_uav2_pose)

        # prepare magnitude diff string
        s_magnitude = """UWB-UAV1 \n
            UAV1-UWB-T265: {} \n
            UAV1-UWB-ALOAM: {}... \n

            UWB-UAV2 \n
            UAV2-UWB-T265: {}\n
            UAV2-UWB-ALOAM: {}\n

            UWB-UAV2 \n
            UAV2-UWB-T265: {}\n
            UAV2-UWB-ALOAM: {}\n
            """.format(0, 0, 0, 0, uwb_t265_magdiff_uav1, uwb_t265_magdiff_uav2)
        
        # Publish everything 
        magnitude_diff_publisher.publish(s_magnitude)

        # append to numpy array

        # if shutdown detected save array to csv
        rate.sleep()
