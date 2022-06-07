#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import rospy
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

        # Variables
        self.recieved_new_callback_uav2=True
        self.recieved_new_callback_uav1=True
        self.cmd1 = PoseStamped()
        self.cmd2 = PoseStamped()

        self.mode = String()
        self.human_pose = PoseStamped()
        self.follow_me_pos = PoseWithCovarianceStamped() 
        self.uav1_go_there_pos = PoseWithCovarianceStamped()
        self.uav2_go_there_pos = PoseWithCovarianceStamped()
        self.uav1_go_there_pos.pose.pose.position.z=0 #initalise to 0 for takeover
        self.uav2_go_there_pos.pose.pose.position.z=0

        # self.uav1_go_there_yaw = Float64()
        # rospy.Subscriber('/uav1/command/yaw',  Float64, self.uav1_go_there_yaw_callback)

        # Subscribers
        rospy.Subscriber('/hri_mode', String, self.mode_callback)
        rospy.Subscriber('/uav_all/follow_me_target_pose',  PoseWithCovarianceStamped, self.position_callback)
        rospy.Subscriber('/uav1/command/pose',  PoseWithCovarianceStamped, self.uav1_go_there_callback)
        rospy.Subscriber('/uav2/command/pose',  PoseWithCovarianceStamped, self.uav2_go_there_callback)

        # Publishers
        # Publish topic for planner
        uav1_publish_topic = "/uav1/teaming_planner/uwb_assigned_virtual_position"
        uav2_publish_topic = "/uav2/teaming_planner/uwb_assigned_virtual_position"

        # uav1_publish_topic = "/uav1/mavros/setpoint_position/local"
        # uav2_publish_topic = "/uav2/mavros/setpoint_position/local"

        uav1_publisher = rospy.Publisher(uav1_publish_topic , PoseStamped,queue_size=1)
        uav2_publisher = rospy.Publisher(uav2_publish_topic , PoseStamped,queue_size=1)

        rate = rospy.Rate(20.0)

        # Drone position in UWB
        self.uav1_pose_uwb_covar = PoseWithCovarianceStamped()
        self.uav2_pose_uwb_covar = PoseWithCovarianceStamped()
        rospy.Subscriber('UAV1PoseUWB',  PoseWithCovarianceStamped, self.uav1_callback_uwb)
        rospy.Subscriber('UAV2PoseUWB',  PoseWithCovarianceStamped, self.uav2_callback_uwb)

        while not rospy.is_shutdown():

            final_pose_uav1  = PoseStamped()
            final_pose_uav2 = PoseStamped()

            vector_diff_uav1 = PoseStamped()
            vector_diff_uav2 = PoseStamped()

            if self.mode.data=="Follow_Me":
                print("Follow me")
                
                self.uav1_uwb_pose_stamped = PoseStamped()
                self.uav2_uwb_pose_stamped = PoseStamped()
                self.uav1_uwb_pose_stamped.pose = self.uav1_pose_uwb_covar.pose.pose
                self.uav2_uwb_pose_stamped.pose = self.uav2_pose_uwb_covar.pose.pose

                formation_pose_uav1 = PoseStamped()
                formation_pose_uav2 = PoseStamped()
                formation_pose_uav1.pose = self.follow_me_pos.pose.pose 
                formation_pose_uav1.pose.position.z=1

                formation_pose_uav2.pose.position.x=self.follow_me_pos.pose.pose.position.x - 2
                formation_pose_uav2.pose.position.y=self.follow_me_pos.pose.pose.position.y 
                formation_pose_uav2.pose.position.z=1

                formation_pose_uav2.pose.orientation.x=self.follow_me_pos.pose.pose.orientation.x
                formation_pose_uav2.pose.orientation.y=self.follow_me_pos.pose.pose.orientation.y
                formation_pose_uav2.pose.orientation.z=self.follow_me_pos.pose.pose.orientation.z
                formation_pose_uav2.pose.orientation.w=self.follow_me_pos.pose.pose.orientation.w

                self.cmd1 = formation_pose_uav1
                self.cmd2 = formation_pose_uav2

            elif self.mode.data=="Go_There":
                print("GT")
                # If there is go there position sent, go to
                if self.uav1_go_there_pos.pose.pose.position.z != 0 :
                    if self.recieved_new_callback_uav1==True:
                        # fml cant just put an and statement above cause it is a diff logic
                        self.uav1_go_there_pos_in_pose_stamped = PoseStamped()
                        self.uav1_go_there_pos_in_pose_stamped.pose = self.uav1_go_there_pos.pose.pose
                        
                        self.cmd1 = self.uav1_go_there_pos_in_pose_stamped
                        self.cmd1.pose.position.z=1
                        self.recieved_new_callback_uav1 = False

                # If there is no position sent yet, hover at current spot
                else:
                    self.cmd1.pose = self.uav1_pose_uwb_covar.pose.pose

                #-------------------------------------------------------------------------------

                # If there is go there position sent, go to
                if self.uav2_go_there_pos.pose.pose.position.z != 0 :
                    if self.recieved_new_callback_uav2:
                        self.uav2_go_there_pos_in_pose_stamped = PoseStamped()
                        self.uav2_go_there_pos_in_pose_stamped.pose = self.uav2_go_there_pos.pose.pose
                        self.cmd2 = self.uav2_go_there_pos_in_pose_stamped
                        self.cmd2.pose.position.z = 1
                        self.recieved_new_callback_uav2 = False
                # If there is no position sent yet, hover at current spot
                else:
                    self.cmd2.pose = self.uav2_pose_uwb_covar.pose.pose

            else:
                print("No Mode")
                self.cmd1.pose = self.uav1_pose_uwb_covar.pose.pose
                self.cmd2.pose = self.uav2_pose_uwb_covar.pose.pose

            print("cmd1_ap:",[self.cmd1.pose.position.x,self.cmd1.pose.position.y,self.cmd1.pose.position.z])
            print("cmd2_ap:",[self.cmd2.pose.position.x,self.cmd2.pose.position.y,self.cmd2.pose.position.z])

            self.cmd1.header.frame_id = '/odom'
            self.cmd2.header.frame_id = '/odom'
            uav1_publisher.publish(self.cmd1)
            uav2_publisher.publish(self.cmd2)
        
            rate.sleep()


    def position_callback(self,data):
        self.follow_me_pos=data

    def mode_callback(self,data):
        self.mode=data

    def uav1_go_there_callback(self,data):
        self.uav1_go_there_pos=data
        self.recieved_new_callback_uav1=True

    def uav2_go_there_callback(self,data):
        self.uav2_go_there_pos=data
        self.recieved_new_callback_uav2=True

    def uav1_callback_uwb(self,data):
        self.uav1_pose_uwb_covar=data
    
    def uav2_callback_uwb(self,data):
        self.uav2_pose_uwb_covar=data

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

        # print("Pose_stamped_now {} {} {}".format(pose_staped_now.pose.position.x, pose_staped_now.pose.position.y, pose_staped_now.pose.position.z))
        # print("Pose_stamped_previous  {} {} {}".format(pose_stamped_previous.pose.position.x, pose_stamped_previous.pose.position.y, pose_stamped_previous.pose.position.z))
        # print("Vector diff {} {} {}".format(vector_diff.pose.position.x, vector_diff.pose.position.y, vector_diff.pose.position.z))

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
