#!/bin/bash

# date_directory=$1

date_directory=$(cat ~/Diagnosis/record_time.txt)
# prepare file_name
file_name=wifi_stats_$(date "+%F-%T").txt

# prepare file_name
file_name=uav1_to_uav2_ping_$(date "+%F-%T").txt

# file path to save it
save_path=~/Diagnosis/$date_directory/$file_name

touch $save_path
# echo "Saving file to"
# echo "$save_path"
# ping -D 192.168.1.63 >> "$save_path"