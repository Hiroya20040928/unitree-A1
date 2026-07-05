#!/usr/bin/env python3
# NX-side ROS Melodic node. Reads LaserScan and writes the nearest front obstacle
# distance to /tmp/a1_obstacle_front_m, which a1_high_follow_driver already uses.

import math
import os
import time

import rospy
from sensor_msgs.msg import LaserScan

OUT_PATH = "/tmp/a1_obstacle_front_m"

class FrontObstacleWriter:
    def __init__(self):
        self.topic = rospy.get_param("~scan_topic", "/scan")
        self.front_deg = float(rospy.get_param("~front_deg", 25.0))
        self.min_valid = float(rospy.get_param("~min_valid", 0.08))
        self.max_valid = float(rospy.get_param("~max_valid", 5.0))
        self.timeout = float(rospy.get_param("~timeout", 0.6))
        self.last_msg_time = 0.0
        self.last_dist = float("inf")
        self.sub = rospy.Subscriber(self.topic, LaserScan, self.cb, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        rospy.loginfo("front obstacle writer topic=%s front_deg=%.1f out=%s", self.topic, self.front_deg, OUT_PATH)

    def cb(self, msg):
        half = math.radians(self.front_deg)
        best = float("inf")
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < self.min_valid or r > self.max_valid:
                continue
            ang = msg.angle_min + i * msg.angle_increment
            # front sector around angle 0.
            if abs(ang) <= half:
                if r < best:
                    best = r
        self.last_msg_time = time.time()
        self.last_dist = best
        self.write(best)

    def write(self, d):
        tmp = OUT_PATH + ".tmp"
        with open(tmp, "w") as f:
            if math.isfinite(d):
                f.write("%.4f\n" % d)
            else:
                f.write("inf\n")
        os.replace(tmp, OUT_PATH)

    def timer_cb(self, _event):
        # If scan is stale, write inf so the robot does not get stuck on old obstacle.
        if time.time() - self.last_msg_time > self.timeout:
            self.write(float("inf"))

if __name__ == "__main__":
    rospy.init_node("a1_laserscan_obstacle_writer")
    FrontObstacleWriter()
    rospy.spin()
