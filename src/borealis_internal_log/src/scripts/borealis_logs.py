import string
import rospy
import numpy as np 
import pandas as pd
import rosnode
import datetime
import rosgraph

from borealis_node_stats import BorealisNodeStats
from borealis_node_resource import BorealisNodeResource

# Use numpy structured arrays
# Finish numpy node_stats
# Finish borealis_node_resource

class BorealisLogs:
    def __init__(self, fast_loop_rate, slow_loop_rate, path_to_store_logs="~", add_topics_log_to_csv=None):
        
        self.path_to_store_logs = path_to_store_logs + "stats.csv"
        rospy.loginfo("Saving to directory {}".format(path_to_store_logs))
        rospy.loginfo("Fast loop rate is at  {}".format(fast_loop_rate))
        rospy.loginfo("Slow loop rate is at  {}".format(slow_loop_rate))

        # is there a better way to do this? memory wise?
        self.borealis_node_stats = BorealisNodeStats()
        self.borealis_node_resource_list = []
        self.node_name_list = rosnode.get_node_names()
        self.master = rosgraph.Master("")

        # start the timers
        self.slow_timer = rospy.Timer(rospy.Duration(slow_loop_rate), self.slow_loop_cb)
        self.fast_timer = rospy.Timer(rospy.Duration(fast_loop_rate), self.fast_loop_cb)

    def slow_loop_cb(self, timer):
        # Nodes initialized before borealis_internal_log node will be considered as same time as when borealis_internal_log node
        # is initialized. Shud have a better utc_time_now_format but this will do
        tmp_node_name_list = rosnode.get_node_names()
        for node_name in tmp_node_name_list:
            if node_name not in self.borealis_node_stats.node_name:
                time_now = rospy.get_time()
                utc_time_now = datetime.datetime.utcfromtimestamp(time_now)

                self.borealis_node_stats.node_name.append(node_name)
                self.borealis_node_stats.time_of_creation.append(time_now)
                self.borealis_node_stats.utc_time_of_creation.append(utc_time_now)

                new_node_resource = BorealisNodeResource(node_name, self.master.lookupNode(node_name))
                self.borealis_node_resource_list.append(new_node_resource)


    def fast_loop_cb(self, timer):
        for node_resouce in self.borealis_node_resource_list:
            pass
    
    def log_everything_to_csv(self):
        # can use numpy to join them up but documentation seems abit ?? due to array of element of diff type
        data = {'Node Name':self.borealis_node_stats.node_name, 'Time of Creation':self.borealis_node_stats.time_of_creation, 'UTC time of creation':self.borealis_node_stats.utc_time_of_creation}
        df = pd.DataFrame(data)
        df.to_csv(self.path_to_store_logs)

        for node in self.borealis_node_resource_list:
            print(node.node_name)
            print(node.uri_address)


        rospy.loginfo("Shut down initiated logging everything to csv")
