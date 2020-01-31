#!/bin/bash
GREEN='\033[0;32m' # Green Color
YELLOW='\033[0;33m' # Yellow Color
NC='\033[0m' # No Color
printf "${GREEN} *** Use-case test 3 - EStop Pressed started... *** ${NC}\n"
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting the necessary components
roslaunch blackbox_main test_uc3.launch &
sleep 5
rostopic echo /trigger &

#test steps
#simulate an estop press - the trigger condition is data != 0
printf "${YELLOW} *** All 3 EStops are 1 (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 1" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 1" -1 > 0 &
rostopic pub /test3 std_msgs/Int32 "data: 1" -1 > 0

printf "${YELLOW} *** 1 of 3 EStops is 0 (trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 1" -1 > 0 &
rostopic pub /test3 std_msgs/Int32 "data: 1" -1 > 0

printf "${YELLOW} *** 2 of 3 EStops are 0 (trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test3 std_msgs/Int32 "data: 1" -1 > 0

printf "${YELLOW} *** 3 of 3 EStops are 0 (trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test3 std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** All 3 EStops are 1 (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 1" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 1" -1 > 0 &
rostopic pub /test3 std_msgs/Int32 "data: 1" -1 > 0

#assertions check


#stopping the test setup
printf "${YELLOW} *** Stopping all components... *** ${NC}\n"
pkill -f roslaunch
pkill -f rostopic
sleep 2
printf "${GREEN}*** Use-case test 3 - EStop Pressed finished. All processes killed. *** ${NC}\n"
