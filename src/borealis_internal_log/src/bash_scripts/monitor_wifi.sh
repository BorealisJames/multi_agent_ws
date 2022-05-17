#!/bin/bash
diagnostic_directory=~/Diagnosis;
path_file_name=/wifi_stats_$(date "+%F-%T").txt;
save_directory=$diagnostic_directory$path_file_name
echo "Saving file to"
echo $save_directory
while sleep 1; do
    iwconfig >> "$save_directory"
    date >> "$save_directory"
done