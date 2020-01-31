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
import urllib2
import json
import argparse
import ConfigParser, os
import ftplib
import time
from smtplib import SMTP
import datetime
import Queue
import shutil
import os
import sys
from os.path import expanduser

class RosNodeBase(object):
    ## Formats a log message
    # @param type The type of the message. Possibilities: ('e', 'w', 'i'). Others are treated as errors.
    # @param msg The message to be printed.
    # @param node_id The name of the node
    def _formatted_log(self, type, msg, node_id):
        m = '[{0}] {1}'.format(node_id, msg)
        if (type == 'i'):
            rospy.loginfo(m)
        elif (type == 'w'):
            rospy.logwarn(m)
        else:
            rospy.logerr(m)

    ## Adds the namespace (node name) to the given string
    # @param param The ending of the path
    # @return The path with the namespace added
    def _ns(self, param):
            return rospy.get_name() + "/" + param

    ## Checks the config values from rosparam
    #
    # If recommended elements are mising, shows a warning. 
    # If mandatory elements are missing, shows an error end exits the program.
    # Some params are none of these, they already have default values that are acceptable.
    def _check_config(self, node_id, mandatory = [], recommended = []):
        mandatory_missing = False
        ## True, if some recommended parameters are missing
        self._recommended_missing = False
        for attr, value in self.__dict__.iteritems():
            if attr in mandatory and str(value) == "None":
                self._formatted_log('e', 'Mandatory config item not set: ' + attr, node_id)
                mandatory_missing = True
            elif attr in recommended and str(value) == "None":
                self._formatted_log('w', 'Recommended config item not set: ' + attr, node_id)
                self._recommended_missing = True
            else:
                self._formatted_log('i', 'Param loaded: ' + attr + ':' + str(value), node_id)

        if mandatory_missing:
            self._formatted_log('e', 'Mandatory config item(s) missing, exiting.', node_id)
            sys.exit()
