#!/bin/bash
GREEN='\033[0;32m' # Green Color
YELLOW='\033[0;33m' # Yellow Color
NC='\033[0m' # No Color
printf "${GREEN} *** Requirement test - Trigger only for change started... *** ${NC}\n"
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting the necessary components
roslaunch blackbox_main test_trigger_only_for_change.launch &
sleep 5
rostopic echo /trigger &

#test steps
#simulate an error code - the trigger condition is data!=0
printf "${YELLOW} *** Error code 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Error code 331 (trigger)... *** ${NC}\n"
sleep 3
rostopic pub /test std_msgs/Int32 "data: 331" -1 > 0
sleep 10

printf "${YELLOW} *** Error code 331 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 331" -1 > 0
sleep 10

printf "${YELLOW} *** Error code 331 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 331" -1 > 0
sleep 10

printf "${YELLOW} *** Error code 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Error code 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Error code 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** Error code 332 (trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 332" -1 > 0
sleep 10

printf "${YELLOW} *** Error code 0 (no trigger)... *** ${NC}\n"
rostopic pub /test std_msgs/Int32 "data: 0" -1 > 0

#assertions check


#stopping the test setup
printf "${YELLOW} *** Stopping all components... *** ${NC}\n"
pkill -f roslaunch
pkill -f rostopic
sleep 2
printf "${GREEN}*** Requirement test - Trigger only for change finished. All processes killed. *** ${NC}\n"
