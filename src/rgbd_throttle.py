#!/usr/bin/env python
#*********************************************************************
#*
#*  Copyright (c) 2016, Antonio Coratelli
#*
#*  Software License Agreement (BSD License)
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#* 		 the distribution.
#*   * Neither the name of the athor nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#*  Author: Antonio Coratelli <antoniocoratelli@gmail.com>
#*  Last edit: May, 2016
#*********************************************************************/

PKG  = "rgbd_throttle"
NAME = "rgbd_throttle"

import roslib; roslib.load_manifest(PKG)
import rospy
import tf
import time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

class rgbd_throttle:


    def __init__(self):
        
        # initialize ros node
        rospy.init_node(NAME)
        rospy.loginfo("Initializing node '%s' ...", NAME)
        
        # subscribed topics
        self.rgb_info_in   = 'rgb/info_in'
        self.rgb_rect_in   = 'rgb/rect_in'
        self.depth_info_in = 'depth/info_in'
        self.depth_rect_in = 'depth/rect_in'
        
        # published topics
        self.rgb_info_out   = 'rgb/info_out'
        self.rgb_rect_out   = 'rgb/rect_out'
        self.depth_info_out = 'depth/info_out'
        self.depth_rect_out = 'depth/rect_out'
        
        # get ros params
        self.secs = float(rospy.get_param('rate', 1.0)) ** (-1)
        
        # init variables
        self.last_rgb_info = 0
        self.accept_rgb_info   = True
        self.accept_rgb_rect   = False
        self.accept_depth_info = False
        self.accept_depth_rect = False
        
        # echo info
        rospy.loginfo("Time between frames: %s secs", self.secs)
        
        # declare subscribed topics
        rospy.Subscriber(self.rgb_info_in,   CameraInfo, self.callback_rgb_info)
        rospy.Subscriber(self.rgb_rect_in,   Image,      self.callback_rgb_rect)
        rospy.Subscriber(self.depth_info_in, CameraInfo, self.callback_depth_info)
        rospy.Subscriber(self.depth_rect_in, Image,      self.callback_depth_rect)
        
        self.pub_rgb_info   = rospy.Publisher(self.rgb_info_out,   CameraInfo, queue_size=5)
        self.pub_rgb_rect   = rospy.Publisher(self.rgb_rect_out,   Image,      queue_size=5)
        self.pub_depth_info = rospy.Publisher(self.depth_info_out, CameraInfo, queue_size=5)
        self.pub_depth_rect = rospy.Publisher(self.depth_rect_out, Image,      queue_size=5)
        
        rospy.spin()
    
    
    def __del__(self):
        
        rospy.loginfo("Closing node '%s'", NAME)
    
    
    def callback_rgb_info(self, data):
        
        if (data.header.stamp.to_sec() <  self.last_rgb_info):
            self.last_rgb_info = 0
        
        if (data.header.stamp.to_sec() >= self.last_rgb_info + self.secs):
            self.accept_rgb_info   = True
            self.accept_rgb_rect   = False
            self.accept_depth_info = False
            self.accept_depth_rect = False
        
        if (self.accept_rgb_info):
            # rospy.loginfo("Publishing rgb_info %s", data.header.stamp.to_sec())
            self.pub_rgb_info.publish(data)
            self.last_rgb_info     = data.header.stamp.to_sec()
            self.accept_rgb_info   = False
            self.accept_rgb_rect   = True
            self.accept_depth_info = True
            self.accept_depth_rect = False
    
    def callback_rgb_rect(self, data):
        
        if (self.accept_rgb_rect):
            # rospy.loginfo("Publishing rgb_rect %s", data.header.stamp.to_sec())
            self.pub_rgb_rect.publish(data)
            self.accept_rgb_rect = False
    
    
    def callback_depth_info(self, data):
        
        if (self.accept_depth_info):
            # rospy.loginfo("Publishing depth_info %s", data.header.stamp.to_sec())
            self.pub_depth_info.publish(data)
            self.accept_depth_info = False
            self.accept_depth_rect = True
    
    
    def callback_depth_rect(self, data):
        
        if (self.accept_depth_rect):
            # rospy.loginfo("Publishing depth_rect %s", data.header.stamp.to_sec())
            self.pub_depth_rect.publish(data)
            self.accept_depth_rect = False
    
    
### MAIN #######################################################################

if __name__ == "__main__":

    c = rgbd_throttle()

