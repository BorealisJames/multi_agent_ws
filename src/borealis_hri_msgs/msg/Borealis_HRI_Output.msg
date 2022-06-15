#Array of poses (includes Distract Pose, Gun Pose, Follow_Me Pose, Idle Pose )
geometry_msgs/PoseArray uav_pose_array

#List of uav states (i.e. Go_There, Follow_Me, Sweep, Distract, Idle)
string[3] uav_state_list

#List of yaw angles relative to uav for uav's to turn to 
float32[3] uav_yaw_list
    