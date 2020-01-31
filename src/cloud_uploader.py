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

from ros_node_base import RosNodeBase
from threading import Thread	
from blackbox_control.srv import GetAvailableRange, Export
from blackbox_main.msg import UploadStatus, UploadStatusList
from blackbox_main.srv import Upload, UploadResponse
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

## Uploads files to an FTP server
class CloudUploader(RosNodeBase):

    ## Initalizes the class
    def __init__(self):
        ## The id of the node, used in logs
        self.node_id = "BBU"
        ## The upload queue
        self.queue = Queue.Queue()
        ## The current task, already removed from the queue.
        self.currentTask = None

    ## Formats a log message
    # @param type The type of the message. Possibilities: ('e', 'w', 'i'). Others are treated as errors.
    # @param msg The message to be printed.
    def formatted_log(self, type, msg):
        RosNodeBase._formatted_log(self, type, msg, self.node_id)

    ## Loads and checks the config values from rosparam
    #
    # If recommended elements are mising, shows a warning. This will make email sending impossible.
    # If mandatory elements are missing, shows an error end exits the program.
    # Some params are none of these, they already have default values that are acceptable.
    def load_config(self):
        # Gets a rospy param with namespace
        def gp(key, def_val):
            return rospy.get_param(RosNodeBase._ns(self, key), def_val)

        #read ftp settings
        ## The host address of the FTP
        self.ftp_host= gp("ftp_host", None)
        ## The password of the FTP
        self.ftp_password= gp("ftp_password", None)
        ## The username of the FTP
        self.ftp_username= gp("ftp_username", None)

        #read smtp settings
        ## The host address of the SMTP server
        self.smtp_host = gp("smtp_host", None)
        ## The username of the SMTP server
        self.smtp_username = gp("smtp_username", None)
        ## The password of the SMTP server
        self.smtp_password = gp("smtp_password", None)
        ## The connection mode of the SMTP server
        self.smtp_connection_mode = gp("smtp_connection_mode", "tls")
        ## The port of the SMTP server
        self.smtp_port = gp("smtp_port", 25)

        ## The path where the archive files are stored
        self.archive_path = gp("archive_path", None)

        #read email settings
        ## The address where the emails should be sent
        self.email_toaddress = gp("email_toaddress", None)
        ## The body of the emails to be sent
        self.email_body = gp("email_body", "Dear user,<br />{0} topic (value: {2}) triggered a new notification for you.<br />The new log file is uploaded to the storage: {1}<br />Best regards,<br />Black Box")
        ## The subject of the emails to be sent
        self.email_subject = gp("email_subject", "New log file uploaded")

        self.formatted_log('i', 'Config loaded')
        
        RosNodeBase._check_config(self, self.node_id, 
                                    mandatory = ["ftp_host", "ftp_username", "archive_path"], 
                                    recommended = ["smtp_host", "smtp_username", "smtp_password", "smtp_port", "email_toadress"])

    ## Sends an email
    # @param archivename The name of the archive file
    # @param topic The name of the topic that caused this mail sending
    # @param value The value of the topic that caused this mail sending
    # @param subject The subject of the email
    # @param body The body of the email
    def send_mail(self, archivename, topic, value, subject, body):
        try:
            #create a message and setup the headers
            msg = MIMEMultipart()
            msg['From'] = self.smtp_username
            msg['To'] = self.email_toaddress
            msg['Subject'] = subject
            msg.attach(MIMEText(body.format(topic, archivename, value), 'html'))
            #connect to the smtp server
            server = SMTP(self.smtp_host, self.smtp_port)
            #use TLS
            server.ehlo()
            server.starttls()
            server.ehlo()
            #login
            server.login(self.smtp_username, self.smtp_password)
            text = msg.as_string()
            #send mail
            res = server.sendmail(self.smtp_username, self.email_toaddress, text)
            if len(res) is not 0:
                self.formatted_log('w', 'SMTP sendmail returned: ' + str(res))
        except Exception as inst:
            self.formatted_log('w', 'Cannot notify the user: {0}, {1}'.format(self.email_toaddress, subject))  
            self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

    ## Uploads a file to an FTP and sends email containing the details
    # @param task The details of the actions to perform
    def upload_and_notify(self, task):
        try:
            #create an ftp session
            session = ftplib.FTP(self.ftp_host, self.ftp_username, str(self.ftp_password))
            path = '{0}/{1}'.format(self.archive_path, task.filename)
            # file to send
            with open(path,'rb') as file:
                # send the file
                session.storbinary('STOR {0}'.format(task.filename), file)    

            #************************************************
            # Uncomment to enable a simulated slow connection
            #self.formatted_log('w', "sleep start")
            #time.sleep(3)
            #self.formatted_log('w', "sleep end")
            #************************************************

            # close the FTP
            session.quit()
            # Checks, if the config values for sending an email are available.
            if not RosNodeBase._recommended_missing:
                #notify the user with email
                self.send_mail(task.filename, task.topic, task.value, self.email_subject, self.email_body) 

        except Exception as inst:
            self.formatted_log('w', "Cannot upload file: " + task.filename)
            self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

            # Checks, if the config values for sending an email are available.
            if not RosNodeBase._recommended_missing:
                mail_subject = "Upload error"
                mail_body = "Dear user,<br />{0} topic (value: {2}) triggered a new notification for you.<br />\
                        We are sorry, but BlackBox can not connect to the configured FTP and can not upload the file: {1}<br />\
                        Best regards,<br />Black Box"
                #notify the user with email
                self.send_mail(task.filename, task.topic, task.value, mail_subject, mail_body) #notify the user with email

    ## Upload Task class
    #
    # Contains information about an upload task, as topic, value, filename
    class UploadTask(object):
        ## Initializes the UploadTask object
        # @param topic The topic that caused the upload
        # @param value The value of the topic
        # @param filename The name of the file to be uploaded
        def __init__(self, topic, value, filename):
            ## The topic that caused the upload
            self.topic = topic
            ## The value of the topic
            self.value = value
            ## The name of the file to be uploaded
            self.filename = filename

    ## Sets the current task
    # @param task The new value of the property
    def set_current_task(self, task):
        self.currentTask = task

    ## Worker method for the uploader thread
    def upload_worker(self):
        # run until the node is shut down 
        while not rospy.is_shutdown():
            # set the currentTask to None, to have no completed task until 
            # the thread is blocked by waiting for the new task to come
            self.set_current_task(None)
            #get the first item in the queue - block if there is none
            try:
                self.set_current_task(self.queue.get(True, 1))
            except Exception as ex:
                continue

            #upload to the ftp and notify the users
            self.upload_and_notify(self.currentTask)

    ## Callback function for the manual upload service
    #
    # The consumption of the queue is running on another thread, this function only adds to the queue
    # @param data The data of the service call
    def upload_callback(self, data):
        try:
            #add new task to the queue
            t = CloudUploader.UploadTask(data.topic, data.value, data.filename)
            self.queue.put(t)

            self.formatted_log('i', 'New task added to the queue: ' + str(t))
            #self.formatted_log('w', 'Current queue length: ' + str(len(self.queue.queue) + (1 if self.currentTask is not None else 0)))

        except Exception as inst:
            self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

        return UploadResponse()

    ## Callback function for the load service
    #
    # Load a selected file from the cloud to the local archive
    # @param data The data of the service call
    def load_callback(self, data):
        try:
            # create an ftp session
            session = ftplib.FTP(self.ftp_host, self.ftp_username, str(self.ftp_password))
            # create the local file name
            local_filename = '{0}{1}'.format(self.archive_path, "black_box_import.zip")
            # retrieve the file
            session.retrbinary("RETR {0}".format(data.filename), open(local_filename,'wb').write) 
            # end the ftp session
            session.quit()

        except Exception as inst:
            self.formatted_log('w', str(type(inst)) + ' ' + str(inst))

        return UploadResponse()

    ## Working method of the status_pub thread
    def status_pub_worker(self):
        # run until the node is shut down
        while not rospy.is_shutdown():
            # every second a list of upload statuses are published
            usl = UploadStatusList()
            # convert the queue to a list, so we can read it without consuming it
            ql = list(self.queue.queue)
            # if the current task is not None, add it to the list
            # this is needed, because the current task is not part of the queue anymore
            if (self.currentTask is not None):
                ql.append(self.currentTask)

            # assemble the list
            for i in ql:
                us = UploadStatus()
                us.uploading = i == self.currentTask
                us.error = ""
                us.file_name = i.filename
                usl.statusList.append(us)

            # publish the list
            self.status_publisher.publish(usl)
            #wait 1s before starting again
            time.sleep(1)

    ## Starts the node activities
    #
    # Advertises services and topics, starts the threads.
    def start(self):
        try:
            # Loads the config values from rosparam
            self.load_config()

            # advertise upload service
            rospy.Service(RosNodeBase._ns(self, 'upload'), Upload, self.upload_callback)
            # advertise load service
            rospy.Service(RosNodeBase._ns(self, 'load'), Upload, self.load_callback)
            ## The status publisher object
            # advertise the status topic
            self.status_publisher = rospy.Publisher(RosNodeBase._ns(self, 'status'), UploadStatusList, queue_size=1)

            ## status update thread
            status_thread = Thread(target = self.status_pub_worker, args=())
            status_thread.start()

            ## uploader thread
            upload_thread = Thread(target = self.upload_worker, args=())
            upload_thread.start()
            
            rospy.spin()

        except Exception as inst:
            self.formatted_log('e', str(type(inst)) + ' ' + str(inst))


## Main function
if __name__ == '__main__':
    try:
        rospy.init_node('cloud_uploader', anonymous=True, log_level=rospy.INFO)
        
        cu = CloudUploader()
        cu.start()

    except Exception as inst:
        rospy.logerr('[BBU] ' + str(type(inst)) + ' ' + str(inst))
