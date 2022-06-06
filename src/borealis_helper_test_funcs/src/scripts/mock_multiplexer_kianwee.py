#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import os
import rospy
import math
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import *
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String

class transform():

    def __init__(self):

        # Mode
        self.mode = String()
        rospy.Subscriber('/hri_mode', String, self.mode_callback)

        # Human pose
        self.human_pose = PoseStamped()
        rospy.Subscriber('/uav2/mavros/local_position/pose',  PoseStamped, self.uav2_callback)

        # Follow Me
        self.follow_me_pos = PoseWithCovarianceStamped() ########################
        rospy.Subscriber('/uav_all/follow_me_target_pose',  PoseWithCovarianceStamped, self.position_callback)

        # Go There
        self.uav1_go_there_pos = PoseWithCovarianceStamped()
        rospy.Subscriber('/uav1/command/pose',  PoseWithCovarianceStamped, self.uav1_go_there_callback)
        # self.uav1_go_there_yaw = Float64()
        # rospy.Subscriber('/uav1/command/yaw',  Float64, self.uav1_go_there_yaw_callback)
        self.uav2_go_there_pos = PoseWithCovarianceStamped()
        rospy.Subscriber('/uav2/command/pose',  PoseWithCovarianceStamped, self.uav2_go_there_callback)

        # Publish topic for planner
        uav1_publish_topic = "/uav1/teaming_planner/assigned_virtual_position"
        uav2_publish_topic = "/uav2/teaming_planner/assigned_virtual_position"

        # uav1_publish_topic = "/uav1/mavros/setpoint_position/local"
        # uav2_publish_topic = "/uav2/mavros/setpoint_position/local"

        uav1_publisher = rospy.Publisher(uav1_publish_topic , PoseStamped,queue_size=1)
        uav2_publisher = rospy.Publisher(uav2_publish_topic , PoseStamped,queue_size=1)

        # Local Position Callback
        self.uav1_pos = PoseStamped()
        rospy.Subscriber('/uav1/mavros/local_position/pose',  PoseStamped, self.uav1_callback)
        self.uav2_pos = PoseStamped()
        rospy.Subscriber('/uav2/mavros/local_position/pose',  PoseStamped, self.uav2_callback)

        self.uav1_go_there_pos.pose.pose.position.z=0 #initalise to 0 for takeover
        self.uav2_go_there_pos.pose.pose.position.z=0

        rate = rospy.Rate(20.0)

        # changes to be implemented functions are at the bottom
        # Local Position Callback in UWB
        self.uav1_pose_uwb = PoseWithCovarianceStamped()
        rospy.Subscriber('UAV1PoseUWB',  PoseWithCovarianceStamped, self.uav1_callback_uwb)
        self.uav2_pose_uwb = PoseWithCovarianceStamped()
        rospy.Subscriber('UAV2PoseUWB',  PoseWithCovarianceStamped, self.uav2_callback_uwb)

        while not rospy.is_shutdown():

            # print(self.mode.data)

            cmd1 = PoseStamped()
            cmd2 = PoseStamped()
            final_pose_uav1  = PoseStamped()
            final_pose_uav2 = PoseStamped()

            vector_diff_uav1 = PoseStamped()
            vector_diff_uav2 = PoseStamped()
            if self.mode.data=="Follow_Me":
                print("FM")

                # dummy variables meant to be replaced by formation generator. 
                # formation is generated from human pose which is UWB frame, therefore formation produced in UWB frame (?)
                formation_pose_uav1 = PoseStamped()
                formation_pose_uav2 = PoseStamped()

                # Take goal in UWB frame - Pose in UWB drone frame
                # Take this vector
                # Add this vector to the goal in t265 frame
                vector_diff_uav1 = self.pose_diff(self.uav1_pose_uwb, formation_pose_uav1)
                final_pose_uav1 = self.pose_addition(vector_diff_uav1, self.uav1_pos)

                vector_diff_uav2 = self.pose_diff(self.uav2_pose_uwb, formation_pose_uav2)
                final_pose_uav2 = self.pose_addition(vector_diff_uav2, self.uav2_pos)

                cmd1 = final_pose_uav1
                cmd2 = final_pose_uav2

            elif self.mode.data=="Go_There":
                print("GT")
                # If there is go there position sent, go to
                if self.uav1_go_there_pos.pose.pose.position.z != 0:
                    self.uav1_go_there_pos_in_pose_stamped = PoseStamped()
                    self.uav1_go_there_pos_in_pose_stamped.pose = self.uav1_go_there_pos.pose.pose

                    # Take goal in UWB frame - Pose in UWB drone frame
                    # Take this vector
                    # Add this vector to the goal in t265 frame
                    
                    self.uwbpose_stamped = PoseStamped()
                    self.uwbpose_stamped = self.uav1_pose_uwb.pose
                    self.uwbpose_stamped.pose.orientation.w = 1

                    # print("uwbpose_stamped_covariance {} {} {}".format(self.uav1_pose_uwb.pose.pose.position.x, self.uav1_pose_uwb.pose.pose.position.y, self.uav1_pose_uwb.pose.pose.position.z))
                    # print("uwbpose_stamped {} {} {}".format(self.uwbpose_stamped.pose.position.x, self.uwbpose_stamped.pose.position.y, self.uwbpose_stamped.pose.position.z))
                    # print("uav1 pose in t265 {} {} {}".format(self.uav1_pos.pose.position.x, self.uav1_pos.pose.position.y, self.uav1_pos.pose.position.z))

                    vector_diff_uav1 = self.pose_diff(self.uwbpose_stamped, self.uav1_go_there_pos_in_pose_stamped)
                    final_pose_uav1 = self.pose_addition(vector_diff_uav1, self.uav1_pos)

                    # print("fking kek")
                    # print(vector_diff_uav1.pose.position.y)

                    cmd1 = final_pose_uav1
                    cmd1.pose.position.z=1

                # If there is no position sent yet, hover at current spot
                else:
                    cmd1.pose.position.x=self.uav1_pos.pose.position.x
                    cmd1.pose.position.y=self.uav1_pos.pose.position.y
                    cmd1.pose.position.z=self.uav1_pos.pose.position.z

                    cmd1.pose.orientation.x=self.uav1_pos.pose.orientation.x
                    cmd1.pose.orientation.y=self.uav1_pos.pose.orientation.y
                    cmd1.pose.orientation.z=self.uav1_pos.pose.orientation.z
                    cmd1.pose.orientation.w=self.uav1_pos.pose.orientation.w

                #-------------------------------------------------------------------------------

                # If there is go there position sent, go to
                if self.uav2_go_there_pos.pose.pose.position.z != 0:

                    self.uav2_go_there_pos_in_pose_stamped = PoseStamped()
                    self.uav2_go_there_pos_in_pose_stamped.pose = self.uav2_go_there_pos.pose.pose
                    self.uwbpose_stamped2 = PoseStamped()
                    self.uwbpose_stamped2 = self.uav2_pose_uwb.pose
                    self.uwbpose_stamped2.pose.orientation.w = 1

                    # Take goal in UWB frame - Pose in UWB drone frame
                    # Take this vector
                    # Add this vector to the goal in t265 frame
                    vector_diff_uav2 = self.pose_diff(self.uwbpose_stamped2, self.uav2_go_there_pos_in_pose_stamped)
                    final_pose_uav2 = self.pose_addition(vector_diff_uav2, self.uav2_pos)

                    cmd2 = final_pose_uav2
                    cmd2.pose.position.z = 1
                # If there is no position sent yet, hover at current spot
                else:
                    cmd2.pose.position.x=self.uav2_pos.pose.position.x
                    cmd2.pose.position.y=self.uav2_pos.pose.position.y
                    cmd2.pose.position.z=self.uav2_pos.pose.position.z

                    cmd2.pose.orientation.x=self.uav2_pos.pose.orientation.x
                    cmd2.pose.orientation.y=self.uav2_pos.pose.orientation.y
                    cmd2.pose.orientation.z=self.uav2_pos.pose.orientation.z
                    cmd2.pose.orientation.w=self.uav2_pos.pose.orientation.w

            else:
                print("No Mode")

                cmd1.pose.position.x=self.uav1_pos.pose.position.x
                cmd1.pose.position.y=self.uav1_pos.pose.position.y
                cmd1.pose.position.z=self.uav1_pos.pose.position.z

                cmd1.pose.orientation.x=self.uav1_pos.pose.orientation.x
                cmd1.pose.orientation.y=self.uav1_pos.pose.orientation.y
                cmd1.pose.orientation.z=self.uav1_pos.pose.orientation.z
                cmd1.pose.orientation.w=self.uav1_pos.pose.orientation.w

                cmd2.pose.position.x=self.uav2_pos.pose.position.x
                cmd2.pose.position.y=self.uav2_pos.pose.position.y
                cmd2.pose.position.z=self.uav2_pos.pose.position.z

                cmd2.pose.orientation.x=self.uav2_pos.pose.orientation.x
                cmd2.pose.orientation.y=self.uav2_pos.pose.orientation.y
                cmd2.pose.orientation.z=self.uav2_pos.pose.orientation.z
                cmd2.pose.orientation.w=self.uav2_pos.pose.orientation.w

            print("1_diff:",[vector_diff_uav1.pose.position.x,vector_diff_uav1.pose.position.y,cmd1.pose.position.z, vector_diff_uav1.pose.orientation.x, vector_diff_uav1.pose.orientation.y, vector_diff_uav1.pose.orientation.z, vector_diff_uav1.pose.orientation.w])
            print("2_diff:",[vector_diff_uav2.pose.position.x,vector_diff_uav2.pose.position.y,vector_diff_uav2.pose.position.z, vector_diff_uav2.pose.orientation.x, vector_diff_uav2.pose.orientation.y, vector_diff_uav2.pose.orientation.z, vector_diff_uav2.pose.orientation.w])

            print("1_uwb:",[cmd1.pose.position.x,cmd1.pose.position.y,cmd1.pose.position.z, cmd1.pose.orientation.x, cmd1.pose.orientation.y, cmd1.pose.orientation.z, cmd1.pose.orientation.w])
            print("2_uwb:",[cmd2.pose.position.x,cmd2.pose.position.y,cmd2.pose.position.z, cmd2.pose.orientation.x, cmd2.pose.orientation.y, cmd2.pose.orientation.z, cmd2.pose.orientation.w])
            cmd1.header.frame_id = '/odom'
            cmd2.header.frame_id = '/odom'
            uav1_publisher.publish(cmd1)
            uav2_publisher.publish(cmd2)

            rate.sleep()


    def position_callback(self,data):
        self.follow_me_pos=data

    def mode_callback(self,data):
        self.mode=data

    def uav1_go_there_callback(self,data):
        self.uav1_go_there_pos=data

    def uav2_go_there_callback(self,data):
        self.uav2_go_there_pos=data

    def uav1_callback(self,data):
        self.uav1_pos=data
    
    def uav2_callback(self,data):
        self.uav2_pos=data

    def uav1_callback_uwb(self,data):
        self.uav1_pose_uwb=data
    
    def uav2_callback_uwb(self,data):
        self.uav2_pose_uwb=data


    # rotation
    # Relative rotation q_r from q_1 to q_2
    # q_2 = q_r*q_1
    # therefore q_r = q_2*q_1_inverse
    def pose_diff(self, pose_stamped_previous, pose_staped_now):
        q1_inv = np.zeros(4)
        q2 = np.zeros(4)
        vector_diff = PoseStamped()

        # position vector diff 
        vector_diff.pose.position.x = pose_staped_now.pose.position.x - pose_stamped_previous.pose.position.x
        vector_diff.pose.position.y = pose_staped_now.pose.position.y - pose_stamped_previous.pose.position.y
        vector_diff.pose.position.z = pose_staped_now.pose.position.z - pose_stamped_previous.pose.position.z

        print("Pose_stamped_now {} {} {}".format(pose_staped_now.pose.position.x, pose_staped_now.pose.position.y, pose_staped_now.pose.position.z))
        print("Pose_stamped_previous  {} {} {}".format(pose_stamped_previous.pose.position.x, pose_stamped_previous.pose.position.y, pose_stamped_previous.pose.position.z))
        print("Vector diff {} {} {}".format(vector_diff.pose.position.x, vector_diff.pose.position.y, vector_diff.pose.position.z))

        q1_inv[0] = pose_stamped_previous.pose.orientation.x
        q1_inv[1] = pose_stamped_previous.pose.orientation.y
        q1_inv[2] = pose_stamped_previous.pose.orientation.z
        q1_inv[3] = -pose_stamped_previous.pose.orientation.w # Negate for inverse

        q2[0] = pose_staped_now.pose.orientation.x
        q2[1] = pose_staped_now.pose.orientation.y
        q2[2] = pose_staped_now.pose.orientation.z
        q2[3] = pose_staped_now.pose.orientation.w

        qr = quaternion_multiply(q2, q1_inv)

        vector_diff.pose.orientation.x = qr[0]
        vector_diff.pose.orientation.y = qr[1]
        vector_diff.pose.orientation.z = qr[2]
        vector_diff.pose.orientation.w = qr[3]

        return vector_diff

    def pose_addition(self, vector_pose_stamped_to_add, pose_staped_now):
        q_rot = np.zeros(4)
        q_origin = np.zeros(4)
        new_pose_stamped = PoseStamped()

        # position vector diff 
        new_pose_stamped.pose.position.x = pose_staped_now.pose.position.x + vector_pose_stamped_to_add.pose.position.x
        new_pose_stamped.pose.position.y = pose_staped_now.pose.position.y + vector_pose_stamped_to_add.pose.position.y
        new_pose_stamped.pose.position.z = pose_staped_now.pose.position.z + vector_pose_stamped_to_add.pose.position.z


        # print("pose stamped now function y")
        # print(pose_staped_now.pose.position.y)
        # print("vector pose stamped function y")
        # print(vector_pose_stamped_to_add.pose.position.y)
        # print("insie function y")
        # print(new_pose_stamped.pose.position.y)

        q_origin[0] = pose_staped_now.pose.orientation.x
        q_origin[1] = pose_staped_now.pose.orientation.y
        q_origin[2] = pose_staped_now.pose.orientation.z
        q_origin[3] = pose_staped_now.pose.orientation.w

        q_rot[0] = vector_pose_stamped_to_add.pose.orientation.x
        q_rot[1] = vector_pose_stamped_to_add.pose.orientation.y
        q_rot[2] = vector_pose_stamped_to_add.pose.orientation.z
        q_rot[3] = vector_pose_stamped_to_add.pose.orientation.w

        q_new = quaternion_multiply(q_rot, q_origin)

        new_pose_stamped.pose.orientation.x = q_new[0]
        new_pose_stamped.pose.orientation.y = q_new[1]
        new_pose_stamped.pose.orientation.z = q_new[2]
        new_pose_stamped.pose.orientation.w = q_new[3]

        return new_pose_stamped

if __name__ == '__main__':
    rospy.init_node('Mock_Multi_Agent')

    node = transform()

    rospy.spin()

## method 2 lmfao

# cmd1.header.frame_id ="/uwb_t265_diff"
# listener = tf.TransformListener()
# try:
#     cmd1_in_t265 = listener.transformPose('/odom', cmd1, rospy.Time(0)) # target frame, source frame
# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#     print("lmfao gg tf dont have transform for that moment")
#     continue
# cmd1 = cmd1_in_t265
