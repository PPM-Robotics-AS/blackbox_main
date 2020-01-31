#!/bin/bash
GREEN='\033[0;32m' # Green Color
YELLOW='\033[0;33m' # Yellow Color
NC='\033[0m' # No Color
printf "${GREEN} *** Use-case test 2 - Voltage Above Threshold started... *** ${NC}\n"
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting the necessary components
roslaunch blackbox_main test_uc2.launch &
sleep 5
rostopic echo /trigger &

#test steps
#simulate an error code - the trigger condition is data > 10
printf "${YELLOW} *** Voltage: 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Voltage: 3 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 3" -1 > 0

printf "${YELLOW} *** Voltage: 8 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 8" -1 > 0

printf "${YELLOW} *** Voltage: 11 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 11" -1 > 0

printf "${YELLOW} *** Voltage: 500 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 500" -1 > 0

printf "${YELLOW} *** Voltage: 9 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 9" -1 > 0

printf "${YELLOW} *** Voltage: -15 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: -15" -1 > 0


#assertions check


#stopping the test setup
printf "${YELLOW} *** Stopping all components... *** ${NC}\n"
pkill -f roslaunch
pkill -f rostopic
sleep 2
printf "${GREEN}*** Use-case test 1 - Voltage Above Threshold finished. All processes killed. *** ${NC}\n"
