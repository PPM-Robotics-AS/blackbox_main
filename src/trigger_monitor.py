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
import roslib
import rostopic
import urllib2
import argparse
from ast import literal_eval
import re
import ftplib
import time
import smtplib
import datetime
import Queue
import shutil
import os
import ujson
from os.path import expanduser
#from deepdiff import DeepDiff

from blackbox_main.msg import Trigger
from ros_node_base import RosNodeBase
from threading import Thread	
from blackbox_control.srv import GetAvailableRange, Export
from blackbox_main.srv import Upload, UploadRequest
from std_msgs.msg import Header

## Handles events and triggers exports based on set conditions
class TriggerMonitor(RosNodeBase):

    def __init__(self):
        ## The id of the node, used in logs
        self.node_id = "BBTM"
        ## Dictionary of conditions
        self.conditions = None
        ## True, if an export operation is ongoing. This will prevent other exports to start
        self.exporting = False
        ## Previous values dictionary
        self.prev_values = {}
        ## Previous change times dictionary
        self.prev_change = {}
        

    ## Formats a log message
    # @param type The type of the message. Possibilities: ('e', 'w', 'i'). Others are treated as errors.
    # @param msg The message to be printed.
    def formatted_log(self, type, msg):
        RosNodeBase._formatted_log(self, type, msg, self.node_id)

    ## Loads and checks the config values from rosparam
    #
    # If recommended elements are mising, shows a warning. 
    # If mandatory elements are missing, shows an error end exits the program.
    # Some params are none of these, they already have default values that are acceptable.
    def load_config(self):
        # Gets a rospy param with namespace
        def gp(key, def_val):
            return rospy.get_param(RosNodeBase._ns(self, key), def_val)

        #read config
        home = expanduser("~")

        try:
            js = gp("conditions", None)
            #self.formatted_log('w', js)
            self.conditions = ujson.loads(js)
        except Exception as inst:
            self.formatted_log('e', str(type(inst)) + ' ' + str(inst))

        ## If true, triggers will happen only when the value of the topic has changed
        self.on_change_only = gp("on_change_only", False)
        ## The minimum time required to register a new trigger
        self.trigger_delay = gp("trigger_delay", 1000)
        ## How many seconds before the event should be exported
        self.bag_from = int(gp("bag_from", "5"))
        ## How many seconds after the event should be exported
        self.bag_to = int(gp("bag_to", "5"))
        ## The path where the bag files are stored
        self.bag_path = gp("bag_path", None)

        self.formatted_log('i', 'Config loaded')

        RosNodeBase._check_config(self, self.node_id, mandatory = ["conditions", "bag_path"])

    ## Saves a bag file and notify the users
    # @param data The published data of the topic
    # @param path The path of the topic
    def variable_changed(self, data, path):
        ## the current time
        now = datetime.datetime.now()
        ## time since the last accepted change
        time_since = datetime.timedelta(weeks=9) if path not in self.prev_change else now - self.prev_change[path]

        ## check if the value of the topic has changed
        changed = path not in self.prev_values or self.prev_values[path] != data
        # save the current value
        self.prev_values[path] = data

        # if the exporting is already being done, then the current event will be part of it anyways
        if self.exporting:
            return

        # if the on_change_only mode is on, and there is no change, return
        if self.on_change_only and not changed:
            return

        # if the time since the last change is less than the minimum, return
        if time_since.total_seconds() * 1000 < self.trigger_delay:
            return

        # save the current time
        self.prev_change[path] = now

        # make the values of other topics available in the conditions
        topics = self.prev_values 

        #check all conditions for a topic
        for key, condition in self.conditions.items():
            try:
                if key == path and eval(condition):
                    self.exporting = True
                    self.export(data, path)

            except Exception as inst:
                self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

        self.exporting = False

    ## Returns the result of the get_available_range ROS service
    # @return The result of the get_available_range ROS service
    def call_get_available_range(self):
        return rospy.ServiceProxy('/black_box/get_available_range', GetAvailableRange)()

    ## Returns the result of the export ROS service
    # @return The result of the export ROS service
    def call_export(self, begin, end):
        return rospy.ServiceProxy('/black_box/export', Export)(begin, end)

    ## Calls the upload service
    #
    # @param file_name The name of the file
    # @param data The new value of the topic
    # @param path The path of the topic
    def call_upload(self, file_name, data, path):
        rospy.ServiceProxy('/cloud_uploader/upload', Upload)(file_name, ujson.dumps(data), path)

    ## Publishes the trigger event data
    #
    # @param file_name The name of the file
    # @param path The path of the topic
    # @param data The new value of the topic
    # @param begin The begin of the exported data
    # @param end The end of the exported data
    def publish_trigger(self, file_name, path, data, begin, end):
        self.trigger_publisher.publish(
                Trigger( 
                    filename = file_name, 
                    topic = path,
                    json = ujson.dumps(data),
                    start = begin,
                    end = end
            ))

    ## Exports the data of the topic change
    #
    # @param data The value of the topic
    # @param path The path of the topic
    def export(self, data, path):
        self.formatted_log('i', "Triggered: {0}, value: {1}".format(path, data))

        #get range
        range_res = self.call_get_available_range()
        
        #parsed ranges
        export_begin = max(range_res.begin, int(range_res.end - self.bag_from))
        export_end = int(range_res.end + self.bag_to)

        #wait N sec after the trigger to export    
        time.sleep(self.bag_to)

        #start export
        exp_res = self.call_export(export_begin, export_end)
        #rospy.loginfo("[ INFO] [ BBU - {3}] Export response: {0}, from: {1}, to: {2}".format(exp_res, export_begin, export_end, time.time()))

        if (exp_res.file_name == ""):
            self.formatted_log('w', "The export failed, nothing to upload ({0} - {1}).".format(export_begin, export_end))
        else:
            self.publish_trigger(exp_res.file_name, path, data, export_begin, export_end)

            try:
                self.call_upload(exp_res.file_name, data, path)
            except rospy.ServiceException as inst:
                    self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

    ## Tries to subscribe to a topic
    #
    # The subscribtion will be done, if the topic is presented so ROS can provide its type.
    # @param path The path of the topic
    def try_subscribe(self, path):
        #dynamically get the topic type
        data_type = rostopic.get_topic_type(path, blocking=False)[0]
        if data_type:
            data_class = roslib.message.get_message_class(data_type)
            #subscribe to the topic
            rospy.Subscriber(path, data_class, callback = self.variable_changed, callback_args = path)
            return True

        return False

    ## Subscribes to a topic
    #
    # If the topic is not advertised by the time of starting this method, it will keep trying indefinitely
    # @param path The path of the topic
    def subscribe(self, path):
        hz = rospy.Rate(10)

        while not self.try_subscribe(path) and not rospy.is_shutdown():
            # rospy.loginfo("%s not presented, wait..."%(path))
            hz.sleep()

    ## Starts the node
    def start(self):
        self.load_config()
        
        #do not listen if black_box node is not running
        rospy.wait_for_service('/black_box/get_available_range')
        
        ## Topic to publish triggered events
        self.trigger_publisher = rospy.Publisher("trigger", Trigger, latch=True, queue_size=1)
        #read subscribes
        for path, condition in self.conditions.items():
            #subscribe to each topic on a different thread, thus: if some topic is not presented
            #we can listen for the others
            t = Thread(target = self.subscribe, args = (path,))
            t.start()
    
        #keep running
        rospy.spin()

    
if __name__ == '__main__':
    try:
        rospy.init_node('trigger_monitor', anonymous=False)

        tm = TriggerMonitor()
        tm.start()
    except Exception as inst:
        rospy.logerr('[BBU] ' + str(type(inst)) + ' ' + str(inst))