import rospy
import os
from borealis_logs import BorealisLogs

if __name__ == "__main__":
    rospy.init_node('borealis_internal_log')
    rospy.loginfo("Borealis internal log running")
    fast_loop = 0.5 # Hz
    slow_loop = 1 
    path_to_store_logs = os.path.expanduser('~/Diagnosis/')
    additional_topics = []

    log_handle = BorealisLogs(fast_loop, slow_loop, path_to_store_logs)
    rospy.on_shutdown(log_handle.log_everything_to_csv())

    rospy.spin()
