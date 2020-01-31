#!/bin/bash
GREEN='\033[0;32m' # Green Color
YELLOW='\033[0;33m' # Yellow Color
NC='\033[0m' # No Color
printf "${GREEN} *** Requirement test - Multi topic condition started... *** ${NC}\n"
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting the necessary components
roslaunch blackbox_main test_multi_topic_condition.launch &
sleep 5
rostopic echo /trigger &

#test steps
printf "${YELLOW} *** false-false (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 100" -1 > 0

printf "${YELLOW} *** true-true (trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 100" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** false-false (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 100" -1 > 0

printf "${YELLOW} *** true-false (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 100" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 100" -1 > 0

printf "${YELLOW} *** false-false (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 100" -1 > 0

printf "${YELLOW} *** false-true (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 0" -1 > 0

printf "${YELLOW} *** false-false (no trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 0" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 100" -1 > 0

printf "${YELLOW} *** true-true (trigger)... *** ${NC}\n"
rostopic pub /test1 std_msgs/Int32 "data: 100" -1 > 0 &
rostopic pub /test2 std_msgs/Int32 "data: 0" -1 > 0


#assertions check
sleep 3

#stopping the test setup
printf "${YELLOW} *** Stopping all components... *** ${NC}\n"
pkill -f roslaunch
pkill -f rostopic
sleep 2
printf "${GREEN}*** Requirement test - Multi topic condition finished. All processes killed. *** ${NC}\n"
