#!/usr/bin/env bash
while :
do
    rostopic pub /blackbox_time std_msgs/UInt64 `date +"%s"` -1 &
    sleep 1    
done
