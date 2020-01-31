#!/usr/bin/python

"""
BLACKBOX
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/blackbox

Copyright (c) 2019 PPM Robotics AS

This library is part of BLACKBOX project,
the Focused Technical Project BLACKBOX - an automated trigger-based 
reporting, data recording and playback unit for ROS.

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: http://rosin-project.eu
This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import rospy
import numpy as np
import urllib
import random
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

roboturl = None

def url_to_image(url):
	resp = urllib.urlopen(url)
	image = np.asarray(bytearray(resp.read()), dtype="uint8")
	image = cv2.imdecode(image, cv2.IMREAD_COLOR)
 
	return image

def start():
    global roboturl 

    rospy.init_node('nachi_tp_view', anonymous=True)

    roboturl = rospy.get_param("/nachi_tp_view/roboturl")
    fps = rospy.get_param("/nachi_tp_view/fps", 1)

    tpscreen = rospy.Publisher('/nachi_tp_view/screen', Image, queue_size=100)
    rate = rospy.Rate(fps)
    
    while not rospy.is_shutdown():
        # I want to publish the Canny Edge Image and the original Image
        msg_frame = CvBridge().cv2_to_imgmsg(url_to_image("http://{0}:16734/snapshot?q={1}".format(roboturl, str(random.randint(111111111,999999999)))), encoding="bgr8")
        tpscreen.publish(msg_frame)
        rate.sleep()

if __name__ == "__main__":
    start()


