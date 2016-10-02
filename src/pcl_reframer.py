#!/usr/bin/env python
'''
Copyright (c) 2016, Antonio Coratelli.
Released under BSD 3-Clause License. See 'LICENSE' file.
'''

import rospy
import tf
import time
from threading import Lock
from sensor_msgs.msg import PointCloud2

class pcl_reframer(object):

    def __init__(self):
        rospy.init_node("pcl_reframer")
        self.initialize_lock()
        self.initialize_params()
        self.initialize_publisher()
        self.initialize_subscriber()
        rospy.spin()

    def callback(self, message):
        self.lock.acquire()
        message_mod = message
        message_mod.header.frame_id = self.new_frame_id
        self.pub.publish(message_mod)
        self.lock.release()

    def initialize_params(self):
        self.queue          = rospy.get_param('~queue', 10)
        self.new_frame_id   = rospy.get_param('~new_frame_id',  '/new_frame_id')

    def initialize_publisher(self):
        self.pub = rospy.Publisher('pcl_out', PointCloud2, queue_size=self.queue)

    def initialize_subscriber(self):
        self.sub = rospy.Subscriber('pcl_in', PointCloud2, self.callback)

    def initialize_lock(self):
        self.lock = Lock()

if __name__ == '__main__':
    pclrf = pcl_reframer()
