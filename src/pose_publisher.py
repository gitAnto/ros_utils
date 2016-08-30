#!/usr/bin/env python
import numpy as np
import rospy
import roslib
import tf

from geometry_msgs.msg import Pose

class PosePublisher:

    def __init__(self):
        self.publisher = rospy.Publisher('in', Pose, queue_size=1)
        self.rate = rospy.Rate(5)

    def publish(self):

        msg = Pose()
        msg.position.x = 2000
        msg.position.y = 2000
        msg.position.z = 2000

        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1

        # Publish odometry message
        self.publisher.publish(msg)

        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    p = PosePublisher()
    while not rospy.is_shutdown():
        p.publish()
