#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomToTF:
    def __init__(self):
        self.parent = rospy.get_param("~parent_frame", "odom")
        self.child = rospy.get_param("~child_frame", "base_footprint")
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber("/odom", Odometry, self.cb, queue_size=20)

    def cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.br.sendTransform(
            (p.x, p.y, 0.0),
            (q.x, q.y, q.z, q.w),
            msg.header.stamp,
            self.child,
            self.parent
        )

if __name__ == "__main__":
    rospy.init_node("a1_odom_to_tf")
    OdomToTF()
    rospy.spin()
