#!/usr/bin/env python2

import rospy
import os
import subprocess 
import rospkg 

if __name__ == "__main__":
    rospy.init_node('borealis_bash_sript')
    rospy.loginfo("Borealis bash script logger running")
    rospack = rospkg.RosPack()
    # get the file path for rospy_tutorials

    path = rospack.get_path('borealis_internal_log') + '/src/bash_scripts/'
    # Get environment variables
    drone_number = os.getenv('DRONE_NUMBER')
    print(path + "monitor_wifi")
    p1 = subprocess.Popen([path + "monitor_wifi.sh"], stdout=subprocess.PIPE)
    if drone_number == "1":
        p2 = subprocess.Popen([path + "ping_from_uav1_to_uav2.sh"], stdout=subprocess.PIPE)
    if drone_number == "2":
        p2 = subprocess.Popen([path+ "ping_from_uav2_to_uav1.sh"], stdout=subprocess.PIPE)

    rospy.spin()
