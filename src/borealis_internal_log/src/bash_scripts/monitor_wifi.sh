#!/bin/bash

date_directory=$(cat ~/Diagnosis/record_time.txt)
# prepare file_name
file_name=wifi_stats_$(date "+%F-%T").txt

# file path to save it
save_path=~/Diagnosis/$date_directory/$file_name
touch $save_path
# echo "Saving file to"
# echo "$save_path"
# while sleep 1; do
#     iwconfig >> "$save_path"
#     date >> "$save_path"
# done