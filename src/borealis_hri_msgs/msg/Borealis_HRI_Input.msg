#Header
std_msgs/Header header

#UAV Selection Mode ID (0: By ID 1:BY quantity)
std_msgs/Int32 uav_selection_mode_int

#ID's of UAV selected: 1 and/or 2 and/or 3
int32[3] uav_id_list

#UAV quantity selected
std_msgs/Int32 uav_quantity_int

#HRI Mode ID (0: Nil 1:Go There 2:Follow Me 3:Sweep 4:Distract 5:Idle)
std_msgs/Int32 hri_mode_int

#Distract Target Data
geometry_msgs/Pose distract_target_pose