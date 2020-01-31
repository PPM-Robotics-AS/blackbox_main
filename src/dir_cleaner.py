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

import shutil
import os
import argparse
import time
import rospy
import sys

## Keeps a directory "clean" by limiting the number of files in it
class DirCleaner(object):
    ## The constructor
    def __init__(self):
         pass

    ## Returns a list of files in a directory
    # @param path The directory to be scanned
    # @return The list of files in the directory
    def listdir(self, path):
        return os.listdir(path)

    ## Removes a file from the computer
    # @param path The path to the file
    def remove(self, path):
        os.remove(path)

    ## Scans a directory, removing the files created the first,
    # if their number exceeds the limit
    # @param path The directory to scan and clean
    # @param max_count The maximum number of files to be kept 
    def keep_local_max(self, path, max_count):
        print("Cleaning..."),
        name_list = self.listdir(path)
        full_list = [os.path.join(path, i) for i in name_list]
        time_sorted_list = sorted(full_list, key=os.path.getmtime)

        if len(time_sorted_list) > max_count:
            for i in range(0, len(time_sorted_list) - max_count):
                self.remove(time_sorted_list[i])

        print("cleaned: {0}".format(max(0, len(time_sorted_list) - max_count)))

    ## The worker thread to run the keep_local_max indefinitely
    # @param path The path to work in
    # @param keep_max The max amount of files to keep
    def worker(self, path, keep_max, once = False):
        while not rospy.is_shutdown():
            self.keep_local_max(path, int(keep_max))
            if once:
                return
            time.sleep(10)

    # Parses the parameters coming from the command line
    # @param argv The list of parameters
    # @return The parsed parameters
    @staticmethod
    def parseArgs(argv):
        parser = argparse.ArgumentParser(
                prog='dir_cleaner.py', description='Cleans the given directory and keeps only the given number from the latest files', prefix_chars='-_')
        parser.add_argument('-p', '--path', help='Path to keep clean', required=True)
        parser.add_argument('-m', '--keep_max', default='10',help='Maximum files to keep')
        parser.add_argument('-n', '__name:', default='10',help='Maximum files to keep')
        parser.add_argument('-l', '__log:', default='10',help='Maximum files to keep')
        return parser.parse_args(argv)

def main():
    print "Directory cleaner initalizing..."
    rospy.init_node('dir_cleaner', anonymous=True)
    
    args = DirCleaner.parseArgs(sys.argv[1:])
    dc = DirCleaner()
    
    try:
        dc.worker(args.path, args.keep_max)
    except KeyboardInterrupt:
        pass

## The main function
if __name__ == '__main__':
    main()