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
NAME = "tf_link_map_to_world"

import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import time

class tf_link_map_to_world:

    def __init__(self):
        
        # initialize ros node
        rospy.init_node(NAME)
        rospy.loginfo("Initializing node '%s' ..." % (NAME))

        # get ros params for: ground truth frames
        self.frame_world       = rospy.get_param('frame_world', '/world')
        self.frame_groundtruth = rospy.get_param('frame_groundtruth', '/vicon/puma/puma')

        # get ros params for: relative frame
        self.frame_map = rospy.get_param('frame_map', '/map')

        # get ros params for: other settings
        self.rate           = float(rospy.get_param('rate', 10.0))
        self.retry_interval = float(rospy.get_param('retry_interval', 2.0))

        # initialize transform variables
        self.__transf_translation = None
        self.__transf_rotation    = None

        # initialize tf listener and broadcaster
        self.__tf_listener    = tf.TransformListener()
        self.__tf_broadcaster = tf.TransformBroadcaster()

        # get first transform from world to groundtruth
        self.catch_world_to_groundtruth()

        # start broadcaster
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish_world_to_map()
            r.sleep()

    def __del__(self):
        
        rospy.loginfo("Closing node '%s'" % (NAME))


    def catch_world_to_groundtruth(self):

        # reset transform variables
        self.__transf_translation = None
        self.__transf_rotation    = None

        # wait for first transform from world to groundtruth
        rospy.loginfo("Waiting for transform from '%s' to '%s'" % (self.frame_world, self.frame_groundtruth))

        while not rospy.is_shutdown():
            try:
                self.__tf_listener.waitForTransform(
                    self.frame_world,
                    self.frame_groundtruth,
                    rospy.Time(0),
                    rospy.Duration(self.retry_interval)
                )
                break
            except(tf.Exception):
                continue
        
        # retrieve transform value
        rospy.loginfo("Retrieving transform from '%s' to '%s'" % (self.frame_world, self.frame_groundtruth))

        (self.__transf_translation, self.__transf_rotation) = self.__tf_listener.lookupTransform(
            self.frame_world,
            self.frame_groundtruth,
            rospy.Time(0)
        )

        rospy.loginfo("Caught! Broadcasting /tf ...")


    def publish_world_to_map(self):

        # publish transform only if values do have a meaning
        if self.__transf_translation != None and self.__transf_rotation != None:
            self.__tf_broadcaster.sendTransform(
                self.__transf_translation,
                self.__transf_rotation,
                rospy.Time.now(),
                self.frame_map,
                self.frame_world
            )


### MAIN #######################################################################

if __name__ == "__main__":

    c = tf_link_map_to_world()

