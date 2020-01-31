#!/usr/bin/env bash
#this will render ROS commands and the BlackBox nodes available
source /opt/ros/kinetic/setup.bash
source ~/blackbox_ws/devel/setup.bash

#starting ROSCore
roscore &
sleep 5

#starting ROSBridge for FlexGui communication
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 3

#starting the recording into the bags folder
mkdir -p ~/black_box/bags &> /dev/null
cd ~/black_box/bags
rosbag record -a --split --duration=10&
sleep 1

#[optional]starting the uploader node
#cd ~/black_box
#python blackbox_uploader.py|& tee -a ~/black_box/bbu.log &
rosrun blackbox_main dir_cleaner.py --path ~/black_box/bags/ --keep_max 500 &
#sleep 5

#starting the control node
rosrun blackbox_control blackbox_control &
sleep 2

#[optional]starting the video streamer node
#rosrun web_video_server web_video_server |& tee -a ~/black_box/wvserver.log&
#sleep 5

#[optional]starting the camera nodes
#sh run_forever.sh http://192.168.0.101/video/mjpg.cgi /camera_image_1 &
#sh run_forever.sh http://192.168.0.101/video/mjpg.cgi /camera_image_2 &
#sh run_forever.sh http://192.168.0.101/video/mjpg.cgi /camera_image_3 &
#sh run_forever.sh http://192.168.0.101/video/mjpg.cgi /camera_image_4 &
#sleep 2

#[optional]starting FlexGui in a Chrome
#chromium-browser --app=http://localhost --start-fullscreen &
#sleep 2

echo "Startup ready."
read -n 1