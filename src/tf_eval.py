#!/usr/bin/env python
#/******************************************************************************
# 
# Software License Agreement (BSD 3-Clause)
# 
# Copyright (c) 2016, Antonio Coratelli
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
#******************************************************************************/

PKG  = "ros_utils"
NAME = "tf_eval"

import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import time

import numpy
import csv

class tf_eval:

    def __init__(self):
        
        # initialize ros node
        rospy.init_node(NAME)
        rospy.loginfo("Initializing node '%s' ..." % (NAME))

        # get ros params for frames
        self.frame_world       = rospy.get_param('frame_world',       '/world')
        self.frame_groundtruth = rospy.get_param('frame_groundtruth', '/vicon/puma/puma')
        self.frame_baselink    = rospy.get_param('frame_baselink',    '/base_link')

        # get ros params for csv file
        self.csv_filename   = rospy.get_param('csv_filename', time.strftime('%Y_%m_%d_%H_%M_%S')+'.csv')
        self.csv_delimiter  = rospy.get_param('csv_delimiter', ' ')

        # get ros params for other settings
        self.rate           = float(rospy.get_param('rate', 10.0))
        self.retry_interval = float(rospy.get_param('retry_interval', 2.0))

        # initialize transform variable
        self._reset_transf_variables()

        # initialize csv variables
        rospy.loginfo("Opening output file '%s'" % (self.csv_filename))
        self.__csv_file   = open(self.csv_filename, 'w')
        self.__csv_writer = csv.writer(self.__csv_file, delimiter = self.csv_delimiter)

        # initialize tf listener
        self.__tf_listener = tf.TransformListener()

        # start tf listener
        rospy.loginfo("Listening to /tf ...")
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.catch_transforms()
            self.publish_as_rostopic() # TODO
            self.export_to_csv()
            r.sleep()


    def __del__(self):

        # safely close csv file
        self.__csv_file.flush()
        self.__csv_file.close()

        rospy.loginfo("Closing node '%s'" % (NAME))


    def _reset_transf_variables(self):

        self.__transf_time = None
        self.__transf_er   = None
        self.__transf_gt   = None
        self.__transf_bl   = None


    def _retrieve_transform(self, frame1, frame2):

        (tra, rot) = self.__tf_listener.lookupTransform(frame1, frame2, rospy.Time(0))
        rpy = tf.transformations.euler_from_quaternion(rot)
        return tra + rpy


    def catch_transforms(self):

        self._reset_transf_variables()

        # wait for transform from groundtruth to baselink
        while not rospy.is_shutdown():
            try:
                self.__tf_listener.waitForTransform(
                    self.frame_groundtruth,
                    self.frame_baselink,
                    rospy.Time(0),
                    rospy.Duration(self.retry_interval)
                )
                break
            except(tf.Exception):
                continue
        

        # retrieve transform value
        self.__transf_er   = self._retrieve_transform(self.frame_groundtruth, self.frame_baselink   )
        self.__transf_gt   = self._retrieve_transform(self.frame_world,       self.frame_groundtruth)
        self.__transf_bl   = self._retrieve_transform(self.frame_world,       self.frame_baselink   )
        self.__transf_time = rospy.Time.now().to_sec()


    def publish_as_rostopic(self):
        
       pass # TODO


    def export_to_csv(self):

        # add row to csv file if transforms are valid
        if self.__transf_time != None:
            self.__csv_writer.writerow(
                [self.__transf_time] +
                list(self.__transf_er) +
                list(self.__transf_gt) +
                list(self.__transf_bl)
            )


### MAIN #######################################################################

if __name__ == "__main__":

    c = tf_eval()

