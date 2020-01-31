#!/bin/bash
GREEN='\033[0;32m' # Green Color
YELLOW='\033[0;33m' # Yellow Color
NC='\033[0m' # No Color
printf "${GREEN} *** Requirement test - Trigger delay started... *** ${NC}\n"
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting the necessary components
roslaunch blackbox_main test_trigger_delay_local.launch &
sleep 5
rostopic echo /trigger &

#test steps
#simulate an error code - the trigger condition is data > 10
printf "${YELLOW} *** Voltage: 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Voltage: 3 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 3" -1 > 0

printf "${YELLOW} *** Voltage: 8 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 8" -1 > 0 &
sleep 6

printf "${YELLOW} *** Voltage: 11 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0 &
sleep 6

printf "${YELLOW} *** Voltage: 11 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0 &
sleep 3

printf "${YELLOW} *** Voltage: 11 fast (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0 &
sleep 1

printf "${YELLOW} *** Voltage: 11 fast (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0 &
sleep 2

printf "${YELLOW} *** Voltage: 11 fast (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0 &
sleep 6

printf "${YELLOW} *** Voltage: 500 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 500" -1 > 0 &
sleep 3

printf "${YELLOW} *** Voltage: 500 fast (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 500" -1 > 0 &
sleep 3
#assertions check


#stopping the test setup
printf "${YELLOW} *** Stopping all components... *** ${NC}\n"
pkill -f roslaunch
pkill -f rostopic
sleep 2
printf "${GREEN}*** Requirement test - Trigger delay finished. All processes killed. *** ${NC}\n"
