#!/usr/bin/env python
'''
Copyright (c) 2016, Antonio Coratelli.
Released under BSD 3-Clause License. See 'LICENSE' file.
'''

import rospy
import tf
import time
from tf.msg import tfMessage


class tf_republisher(object):

    def __init__(self):
        rospy.init_node("tf_republisher")
        self.initialize_params()
        self.initialize_publisher()
        self.initialize_subscriber()
        rospy.spin()

    def callback(self, message):
        message_mod = message
        transforms_mod = []
        for t in message.transforms:
            if  self.is_original_frame_id(t.header.frame_id)\
            and self.is_original_child_frame_id(t.child_frame_id):
                t.header.frame_id = self.new_frame_id
                t.child_frame_id  = self.new_child_frame_id
                transforms_mod.append(t)
        if len(transforms_mod) > 0:
            message_mod.transforms = transforms_mod
            self.pub.publish(message_mod)

    def is_original_frame_id(self, frame_id):
        a = frame_id.lstrip('/')
        b = self.original_frame_id.lstrip('/')
        return  a == b

    def is_original_child_frame_id(self, child_frame_id):
        a = child_frame_id.lstrip('/')
        b = self.original_child_frame_id.lstrip('/')
        return a == b

    def initialize_params(self):
        self.queue = rospy.get_param('~queue', 10)
        self.original_frame_id       = rospy.get_param('~original_frame_id',       '/world')
        self.original_child_frame_id = rospy.get_param('~original_child_frame_id', '/vicon/puma/puma')
        self.new_frame_id            = rospy.get_param('~new_frame_id',            '/world_new')
        self.new_child_frame_id      = rospy.get_param('~new_child_frame_id',      '/vicon/puma/puma_new')

    def initialize_publisher(self):
        self.pub = rospy.Publisher('tf_new', tfMessage, queue_size=self.queue)

    def initialize_subscriber(self):
        self.sub = rospy.Subscriber('tf_old', tfMessage, self.callback)

if __name__ == '__main__':
    tfr = tf_republisher()
