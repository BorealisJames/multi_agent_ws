#!/bin/bash
diagnostic_directory=~/Diagnosis;
path_file_name=/uav2_to_uav1_ping_$(date "+%F-%T").txt;
save_directory=$diagnostic_directory$path_file_name
echo "Saving file to"
echo $save_directory
ping -D 192.168.1.63 >> "$save_directory"