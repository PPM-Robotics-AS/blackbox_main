#!/usr/bin/env bash
pkill -f rosbridge_server
killall blackbox_control
killall rosbag
pkill -f blackbox_uploader.py
pkill -f dir_cleaner.py
killall web_video_server
killall chromium-browser
pkill -f run_forever.sh
killall roscore
sleep 5
echo "Shutdown ready."