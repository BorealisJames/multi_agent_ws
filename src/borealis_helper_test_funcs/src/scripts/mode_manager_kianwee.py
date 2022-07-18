#!/usr/bin/env python

"""
Mode Manager listens to inputs and selectively rebroadcast outputs based on the human commands, it also keep tracks of the internal state(mode) of the drone
The multiplexer also implements safety conditions to ensure the uav hovers when no new data is being sent or another intensive node is buffering
The main input is /borealis_hri_output_topic and the Borealis_HRI_Output.msg
And the main output is /command/mode, /command/pose, /hri_mode which is output directly to the motion planner(or mavros setpoints during debugging)
Behaviour
------------
Follow Me & Go There - generate the number of drones in this mode and send the setpoint to the formation planner, wait for the feedback and publish to S&A
Sweep - Stay at current position and yaw to present angles given by this node directly to S&A
Distract - Send target pos and yaw given by human directly to S&A
Nil - use last/previous command
TODO - Maybe add in slowyaw implementation for go_there and before distract 
"""

import rospy
import math
import os
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8, String, Bool, Float32, Int8MultiArray
import numpy as np
from borealis_hri_msgs.msg import Borealis_HRI_Output   
import time


drone_name = str(rospy.get_param('dronename'))
print(drone_name)
drone_number = rospy.get_param('uavnumber')

rate = 5 # Hz, initally 20

class multiplexer():

    def __init__(self):

        global rate
        
        # Distract duration
        self.distract_duration = 5 #s
        self.distract_timer = 0 #dont touch

        self.sweeparr=[]

        # Teaming Planner topics
        self.teaming_pub = rospy.Publisher(drone_name + "/teaming_planner/activate_planner" , Bool,queue_size=1) # pub once
        self.teaming_setpoint = PoseStamped()
        self.prevteaming_setpoint = PoseStamped()
        rospy.Subscriber(drone_name + "/borealis_teaming_planner/assigned_position_output",  PoseStamped, self.teaming_planner_callback)
        # teaming_sub.unregister()
        # teaming_pub.unregister()
        self.num_agents_pub = rospy.Publisher(drone_name + "/number_of_agents_in_team" , Int8MultiArray,queue_size=1) # Total number of drones in go there
        self.drone_formation_publisher = rospy.Publisher(drone_name + "/input_pose_stamped", PoseStamped,queue_size=1)
        self.teaming_enable = True
        rospy.Subscriber("/disable_teaming_planner", Bool, self.teaming_override_callback) # used to override disable the formation planner on the fly
        self.int8array=Int8MultiArray()

        # Output for Sense&Avoid
        self.drone_setpoint_publisher = rospy.Publisher(drone_name + "/control_manager/input", PoseStamped,queue_size=1) # Change to /mavros/setpoint_position/local for direct pub, /control_manager/input , /control_manager/mavros_assigned_virtual_position
        self.drone_mode_publisher = rospy.Publisher(drone_name + "/hri_mode", String,queue_size=1)
        self.setpoint = PoseStamped()
        self.prevsetpoint = PoseStamped()
        rospy.Subscriber(drone_name + "/mavros/setpoint_position/local", PoseStamped, self.setpoint_callback) # To check if the S&A planner has finished planning
        
        # Use Enable this to determine which enable debug message for which stage of planning the drone is at
        planner_status = True # Enable debugging output
        self.planner_status_publisher = rospy.Publisher(drone_name + "/planner_status", String,queue_size=1)
        self.prevplanner_state = ""

        # Local Position Callback
        self.uav_pos = uav()
        rospy.Subscriber(drone_name + '/mavros/local_position/pose',  PoseStamped, self.local_position_callback)

        # Human Input
        rospy.Subscriber('/borealis_hri_output_topic', Borealis_HRI_Output, self.human_callback)
        self.human_command = Borealis_HRI_Output()
        self.prevmode = ""
        self.pos_command = Point()
        self.prevpos_command = Point()
        self.yaw_command = Float32()
        self.prevyaw_command = Float32()
        self.human_command_posestamped = PoseStamped() # Convert into posestamped

        # The problematic thing about this array type is that if there is no command being sent yet, the list index is out of range. Hence, initalise the current position of the uav first to prevent code from crashing
        if self.human_command.uav_state_list == [] or self.human_command.uav_pose_array.poses == []:
            self.mode = "Idle"
            self.pos_command.x = self.uav_pos.x
            self.pos_command.y = self.uav_pos.y
            self.pos_command.z = self.uav_pos.z
            self.yaw_command = euler_from_quaternion([self.uav_pos.rx,self.uav_pos.ry,self.uav_pos.rz,self.uav_pos.rw])[2]
        else:
            self.mode =self.human_command.uav_state_list[drone_number-1]
            self.pos_command=self.human_command.uav_pose_array.poses[drone_number-1].position
            self.yaw_command=self.human_command.uav_yaw_list[drone_number-1]

        self.prevcmd = PoseStamped()
        self.rosrate=rospy.Rate(rate)

        time.sleep(0.1)

        while not rospy.is_shutdown():

            cmd = PoseStamped()
            cmd.header.frame_id="/odom"

            if self.teaming_enable == False:
                self.mock_positions()

            # Send Information to Concensus Global Path or Formation Planner; When process if completed, send new waypoints to Sense & Avoid
            if self.mode=="Follow_Me":

                # Print out debug messages for each stage of planning
                if planner_status == True:
                    self.planner_debug()

                if self.teaming_setpoint.pose.position.z != 0:
                    cmd.pose.position.x=self.teaming_setpoint.pose.position.x
                    cmd.pose.position.y=self.teaming_setpoint.pose.position.y
                    cmd.pose.position.z=self.teaming_setpoint.pose.position.z

                    cmd.pose.orientation.x=self.teaming_setpoint.pose.orientation.x
                    cmd.pose.orientation.y=self.teaming_setpoint.pose.orientation.y
                    cmd.pose.orientation.z=self.teaming_setpoint.pose.orientation.z
                    cmd.pose.orientation.w=self.teaming_setpoint.pose.orientation.w

                else:
                    # rospy.loginfo("No new follow me setpoint sent, hovering")
                    cmd.pose.position.x=self.uav_pos.x
                    cmd.pose.position.y=self.uav_pos.y
                    cmd.pose.position.z=self.uav_pos.z

                    cmd.pose.orientation.x=self.uav_pos.rx
                    cmd.pose.orientation.y=self.uav_pos.ry
                    cmd.pose.orientation.z=self.uav_pos.rz
                    cmd.pose.orientation.w=self.uav_pos.rw

            # Send waypoints from human directly to Sense & Avoid
            elif self.mode=="Go_There":

                # Print out debug messages for each stage of planning
                if planner_status == True:
                    self.planner_debug()

                if self.teaming_setpoint.pose.position.z != 0:
                    cmd.pose.position.x=self.teaming_setpoint.pose.position.x
                    cmd.pose.position.y=self.teaming_setpoint.pose.position.y
                    cmd.pose.position.z=self.teaming_setpoint.pose.position.z

                    cmd.pose.orientation.x=self.teaming_setpoint.pose.orientation.x
                    cmd.pose.orientation.y=self.teaming_setpoint.pose.orientation.y
                    cmd.pose.orientation.z=self.teaming_setpoint.pose.orientation.z
                    cmd.pose.orientation.w=self.teaming_setpoint.pose.orientation.w

                else:
                    # rospy.loginfo("No new go there setpoint sent, hovering")
                    cmd.pose.position.x=self.uav_pos.x
                    cmd.pose.position.y=self.uav_pos.y
                    cmd.pose.position.z=self.uav_pos.z

                    cmd.pose.orientation.x=self.uav_pos.rx
                    cmd.pose.orientation.y=self.uav_pos.ry
                    cmd.pose.orientation.z=self.uav_pos.rz
                    cmd.pose.orientation.w=self.uav_pos.rw


            # Read Yaw command from HRI and send sweep command to Sense & Avoid
            elif self.mode=="Sweep":
                if self.prevmode != "Sweep":
                    cmd.pose.position.x=self.uav_pos.x
                    cmd.pose.position.y=self.uav_pos.y
                    cmd.pose.position.z=self.uav_pos.z
                else:
                    cmd.pose.position.x=self.prevcmd.pose.position.x
                    cmd.pose.position.y=self.prevcmd.pose.position.y
                    cmd.pose.position.z=self.prevcmd.pose.position.z

                q = quaternion_from_euler(0, 0, math.radians(self.slowyaw()))
                cmd.pose.orientation.x=q[0]
                cmd.pose.orientation.y=q[1]
                cmd.pose.orientation.z=q[2]
                cmd.pose.orientation.w=q[3]

            elif self.mode=="Distract":
                cmd.pose.position.x=self.pos_command.x
                cmd.pose.position.y=self.pos_command.y
                cmd.pose.position.z=1.6

                # Grab yaw from command
                q = quaternion_from_euler(0, 0, math.radians(self.yaw_command))
                cmd.pose.orientation.x=q[0]
                cmd.pose.orientation.y=q[1]
                cmd.pose.orientation.z=q[2]
                cmd.pose.orientation.w=q[3]

                if time.time() > self.distract_timer + self.distract_duration:
                    self.mode = "Idle"
                    print("Exiting distract mode")

            # If no new command is sent, use previous command
            elif self.mode=="Nil":
                cmd=self.prevcmd

            else:
                # If the mode is set to idle, grab the first set of coordinates the drone is at and contionusly publish it
                if self.mode=="Idle" and self.prevmode=="Idle":
                    cmd=self.prevcmd
            # If there are no other new commands, set to hover at current position
                else:
                    cmd.pose.position.x=self.uav_pos.x
                    cmd.pose.position.y=self.uav_pos.y
                    cmd.pose.position.z=self.uav_pos.z

                    cmd.pose.orientation.x=self.uav_pos.rx
                    cmd.pose.orientation.y=self.uav_pos.ry
                    cmd.pose.orientation.z=self.uav_pos.rz
                    cmd.pose.orientation.w=self.uav_pos.rw
                
            # Always publish the pose, but only print if 1) the pose differs 
            # self.mode != self.prevmode or
            if self.prevcmd.pose.position.x != cmd.pose.position.x or self.prevcmd.pose.position.y != cmd.pose.position.y or self.prevcmd.pose.position.z != cmd.pose.position.z or self.prevcmd.pose.orientation.w != cmd.pose.orientation.w or self.prevcmd.pose.orientation.x != cmd.pose.orientation.x or self.prevcmd.pose.orientation.y != cmd.pose.orientation.y or self.prevcmd.pose.orientation.z != cmd.pose.orientation.z:
                # and 2) if drone is not hovering on the spot in idle
                if (self.prevmode != "Idle" and self.mode != "Idle") or (self.prevmode != "Nil" and self.mode == "Nil"):
                    print(self.mode,"[%.3f, %.3f, %.3f, %.3f deg]" % (cmd.pose.position.x,cmd.pose.position.y,cmd.pose.position.z, math.degrees(euler_from_quaternion([cmd.pose.orientation.x,cmd.pose.orientation.y,cmd.pose.orientation.z,cmd.pose.orientation.w])[2])))

            self.drone_setpoint_publisher.publish(cmd)

            self.prevcmd = cmd
            self.prevmode=self.mode
            
            self.drone_mode_publisher.publish(self.mode)
            self.drone_formation_publisher.publish(self.human_command_posestamped)
            self.num_agents_pub.publish(self.int8array)

            self.rosrate.sleep()

    # Slows down sweep to a slower predefined speed, function is made to be non-blocking
    # w is angular velocity in degrees per second
    # Not inputting an angle(or putting in 720 degrees) defaults it to a auto sweep mode
    # Take note that both the inputs and the outputs are in DEGREES, not radians
    def slowyaw(self, angle=720, w=20):
        global rate

        yaw = euler_from_quaternion([self.uav_pos.rx, self.uav_pos.ry, self.uav_pos.rz, self.uav_pos.rw])[2]

        # Preset sweep pattern (45,-45,90,-90)
        # TODO, code is quite inelegant because it was copied from previous method, cleanup code
        if self.beginsweep==0 and angle==720:
            self.sweeparr=[]
            
            yaw = math.degrees(yaw)
            print("Inital yaw:",yaw)

            angle = yaw + 90
            print("Yawing to 90, -90, 0")

            print(angle)
            totaldeg=angle-yaw
            numofsteps=round(totaldeg/w*rate)
            if totaldeg > 0:
                for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
            else:
                print(angle - totaldeg/numofsteps)
                for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)
            
            yaw = angle
            angle = yaw-180
            totaldeg=angle-yaw
            numofsteps=round(totaldeg/w*rate)
            print(angle)
            if totaldeg > 0:
                for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
            else:
                print(angle - totaldeg/numofsteps)
                for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

            yaw = angle
            angle = yaw+90
            totaldeg=angle-yaw
            numofsteps=round(totaldeg/w*rate)
            print(angle)
            if totaldeg > 0:
                for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
            else:
                for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

            self.beginsweep=1

        elif self.beginsweep==0 and (yaw - angle) != 0:
            self.sweeparr=[]
            print("Yawing from {0} to {1}".format(yaw,angle))

            totaldeg=angle-yaw
            numofsteps=round(totaldeg/w*rate)
            if totaldeg > 0:
                for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
            else:
                for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)
            self.beginsweep=1
        #     angle = yaw + 45
        #     print("Yawing from {0} to {1}, followed by automated sweep to -45, 90, -90, 0".format(yaw,angle))

        #     print(angle)
        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         print(angle - totaldeg/numofsteps)
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)
            
        #     yaw = angle
        #     angle = yaw-90
        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     print(angle)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         print(angle - totaldeg/numofsteps)
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

        #     yaw = angle
        #     angle = yaw+135
        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     print(angle)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

        #     yaw = angle
        #     angle = yaw-180
        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     print(angle)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

        #     yaw = angle
        #     angle = yaw+90
        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     print(angle)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)

        #     self.beginsweep=1

        # elif self.beginsweep==0 and (yaw - angle) != 0:
        #     self.sweeparr=[]
        #     print("Yawing from {0} to {1}".format(yaw,angle))

        #     totaldeg=angle-yaw
        #     numofsteps=round(totaldeg/w*rate)
        #     if totaldeg > 0:
        #         for i in np.arange(yaw, angle + totaldeg/numofsteps, totaldeg/numofsteps): self.sweeparr.append(i)
        #     else:
        #         for i in np.arange(yaw, angle - totaldeg/numofsteps, -totaldeg/numofsteps): self.sweeparr.append(i)
        #     self.beginsweep=1

            # print(self.sweeparr)
        
        # Should never be invoked, left for debugging
        elif self.beginsweep==0 and (yaw - angle) == 0:
            print("Not sweeping as provided angle is the same as current heading")
            self.mode="Idle"
            return angle
        elif self.beginsweep==0:
            print("Not Sweeping")
            self.mode="Idle"
            return angle
        else:
            if self.sweeparr==[]:
                print("Sweep ended")
                self.mode="Idle"
                return math.degrees(euler_from_quaternion([self.uav_pos.rx,self.uav_pos.ry,self.uav_pos.rz,self.uav_pos.rw])[2]) # return the current position (else it defaults to 0)
        
        desiredyaw=self.sweeparr[0]
        self.sweeparr.pop(0)
        return desiredyaw
            
    def local_position_callback(self,data):
        self.uav_pos.x=data.pose.position.x
        self.uav_pos.y=data.pose.position.y
        self.uav_pos.z=data.pose.position.z
        self.uav_pos.rx=data.pose.orientation.x
        self.uav_pos.ry=data.pose.orientation.y
        self.uav_pos.rz=data.pose.orientation.z
        self.uav_pos.rw=data.pose.orientation.w

    def human_callback(self,msg):
        if msg.uav_state_list == [] or msg.uav_pose_array.poses == []:
            self.mode = "Idle"
            self.pos_command.x = self.uav_pos.x
            self.pos_command.y = self.uav_pos.y
            self.pos_command.z = self.uav_pos.z
            self.yaw_command = euler_from_quaternion([self.uav_pos.rx,self.uav_pos.ry,self.uav_pos.rz,self.uav_pos.rw])[int(drone_number)-1]
            print("Human Message not publishing")
        else:
            self.mode =msg.uav_state_list[drone_number-1]
            self.pos_command=msg.uav_pose_array.poses[drone_number-1].position
            self.yaw_command=msg.uav_yaw_list[drone_number-1]

            # Convert point and yaw into posestamped commands
            self.human_command_posestamped.pose.position.x = self.pos_command.x
            self.human_command_posestamped.pose.position.y = self.pos_command.y
            self.human_command_posestamped.pose.position.z = self.pos_command.z
            q = quaternion_from_euler(0, 0, math.radians(self.yaw_command))
            self.human_command_posestamped.pose.orientation.x=q[0]
            self.human_command_posestamped.pose.orientation.y=q[1]
            self.human_command_posestamped.pose.orientation.z=q[2]
            self.human_command_posestamped.pose.orientation.w=q[3]

        # Initalise one time commands
        if self.mode != self.prevmode or self.prevpos_command != self.pos_command or self.prevyaw_command != self.yaw_command:
            if (self.mode=="Follow_Me" or self.mode=="Go_There"):
                self.teaming_pub.publish(True)
                self.drone_mode_publisher.publish(self.mode)
                # time.sleep(0.2)
                self.drone_formation_publisher.publish(self.human_command_posestamped) # Publish command once to formation planner in follow_me or go_there
            # Start Sweep
            elif self.mode=="Sweep":
                self.beginsweep=0
            elif self.mode=="Distract":
                self.distract_timer = time.time()
            elif self.mode=="Idle":
                rospy.loginfo("Idle Mode")
            elif self.mode=="Nil":
                rospy.loginfo("Nil Mode")
            else:
                print("No Mode or Invalid Mode")

            self.int8array=Int8MultiArray()
            # Check how many drones are in the same mode
            if self.mode == "Follow_Me":
                for i in np.arange(0, len(msg.uav_state_list) -1, 1):
                    if msg.uav_state_list[i] == "Follow_Me": self.int8array.data.append(int(i)+int(1)) # Python array starts from 0
            if self.mode == "Go_There":
                for i in np.arange(0, len(msg.uav_state_list) -1, 1):
                    if msg.uav_state_list[i] == "Go_There": self.int8array.data.append(int(i)+int(1))
            self.num_agents_pub.publish(self.int8array)
        
        # Update previous commands
        self.prevpos_command=self.pos_command
        self.prevyaw_command=self.yaw_command

    # This function enables print statement and a topic to check what stage of planning the drone is at
    def planner_debug(self):

        # Everytime a new human command is sent, the updated positions get sent to Formation Planner
        # The state is then changed to Formation
        if self.pos_command != self.prevpos_command or self.mode != self.prevmode:
            # print("New human command sent to Formation Planner")
            self.prevpos_command = self.pos_command
            self.prevplanner_state = "Formation"
            self.planner_status_publisher.publish("Formation")
        # If the formation planner updates its setpoint, the state is updated to Sense and Avoid
        elif self.teaming_setpoint != self.prevteaming_setpoint and self.prevplanner_state != "S&A":
            # print("Teaming Planner has sent new setpoint to S&A")
            self.prevteaming_setpoint = self.teaming_setpoint
            self.prevplanner_state = "S&A"
            self.planner_status_publisher.publish("S&A")
        # If S&A publishes a new setpoint to the drone
        elif self.setpoint != self.prevsetpoint and self.prevplanner_state != "Moving":
            # print("Sense&Avoid has sent new setpoint to mavros")
            self.prevsetpoint = self.setpoint
            self.prevplanner_state = "Moving"
            self.planner_status_publisher.publish("Moving")
        elif math.sqrt((self.setpoint.pose.position.x - self.uav_pos.x)**2 + (self.setpoint.pose.position.y - self.uav_pos.y)**2 + (self.setpoint.pose.position.z - self.uav_pos.z)**2) < 0.2 and self.prevplanner_state == "Moving":
            # print("Drone has reached setpoint, awaiting new human command")
            self.prevplanner_state = "Idle"
            self.planner_status_publisher.publish("Idle")
            
    # Grab values from the /borealis_hri_output_topic instead of teaming planner for debugging
    def mock_positions(self):
        if self.mode == "Follow_Me" or self.mode == 'Go_There':
            self.teaming_setpoint.pose.position.x = self.pos_command.x
            self.teaming_setpoint.pose.position.y = self.pos_command.y
            self.teaming_setpoint.pose.position.z = self.pos_command.z
            q = quaternion_from_euler(0, 0, math.radians(self.yaw_command))
            self.teaming_setpoint.pose.orientation.x=q[0]
            self.teaming_setpoint.pose.orientation.y=q[1]
            self.teaming_setpoint.pose.orientation.z=q[2]
            self.teaming_setpoint.pose.orientation.w=q[3]

    def uav_go_there_callback(self,data):
        self.uav_go_there_pos=data

    def uav_go_there_yaw_callback(self,data):
        self.uav_go_there_pos=data
    
    def uav_callback(self,data):
        self.uav_pos=data

    def setpoint_callback(self,data):
        self.setpoint=data

    def teaming_planner_callback(self,data):
        self.teaming_setpoint = data

    def teaming_override_callback(self,data):
        if data==1:
            self.teaming_enable=False
            rospy.loginfo_once("Teaming planner disabled")
        if data==0:
            self.teaming_enable=True
            rospy.loginfo_once("Teaming planner Enabled")




# UAV class to hold variables
class uav():

    def __init__(self):
        self.x=0
        self.y=0
        self.z=0
        self.rx=0
        self.ry=0
        self.rz=0
        self.rw=0


if __name__ == '__main__':
    rospy.init_node(drone_name + '_Mode_Manager')

    node = multiplexer()

    rospy.spin()