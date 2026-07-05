#!/usr/bin/env python
# coding: utf-8

from __future__ import print_function

import math
import random
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class WanderObstacleAvoidance(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.control_rate = float(rospy.get_param("~control_rate", 10.0))
        self.scan_timeout_sec = float(rospy.get_param("~scan_timeout_sec", 1.0))

        self.linear_speed = float(rospy.get_param("~linear_speed", 0.10))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.50))
        self.escape_angular_speed = float(rospy.get_param("~escape_angular_speed", 0.80))
        self.forward_clearance = float(rospy.get_param("~forward_clearance", 0.40))
        self.side_clearance = float(rospy.get_param("~side_clearance", 0.32))
        self.emergency_clearance = float(rospy.get_param("~emergency_clearance", 0.30))

        self.random_turn_min_sec = float(rospy.get_param("~random_turn_min_sec", 1.0))
        self.random_turn_max_sec = float(rospy.get_param("~random_turn_max_sec", 2.5))
        self.random_cruise_min_sec = float(rospy.get_param("~random_cruise_min_sec", 3.0))
        self.random_cruise_max_sec = float(rospy.get_param("~random_cruise_max_sec", 8.0))

        self.front_half_angle_deg = float(rospy.get_param("~front_half_angle_deg", 22.0))
        self.side_sector_deg = float(rospy.get_param("~side_sector_deg", 65.0))

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.handle_scan, queue_size=1)

        self.last_scan = None
        self.last_scan_time = 0.0
        self.mode = "waiting_for_scan"
        self.turn_direction = 1.0
        self.random_turn_until = 0.0
        self.random_cruise_until = time.time() + self.sample_cruise_duration()

        rospy.on_shutdown(self.shutdown)

    def handle_scan(self, scan):
        self.last_scan = scan
        self.last_scan_time = time.time()

    def sample_turn_duration(self):
        return random.uniform(self.random_turn_min_sec, self.random_turn_max_sec)

    def sample_cruise_duration(self):
        return random.uniform(self.random_cruise_min_sec, self.random_cruise_max_sec)

    def finite_min(self, values, fallback):
        finite_values = [value for value in values if math.isfinite(value) and value > 0.0]
        if not finite_values:
            return fallback
        return min(finite_values)

    def sector_min(self, scan, start_deg, end_deg):
        if scan is None or not scan.ranges:
            return float("inf")

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        angle = scan.angle_min
        values = []

        for distance in scan.ranges:
            if start_rad <= angle <= end_rad:
                values.append(distance)
            angle += scan.angle_increment

        fallback = scan.range_max if math.isfinite(scan.range_max) and scan.range_max > 0.0 else 30.0
        return self.finite_min(values, fallback)

    def build_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        return cmd

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.status_pub.publish(String(data=mode))
            rospy.loginfo("wander_obstacle_avoidance mode: %s", mode)

    def select_turn_direction(self, left_min, right_min):
        if left_min > right_min + 0.05:
            return 1.0
        if right_min > left_min + 0.05:
            return -1.0
        return random.choice([-1.0, 1.0])

    def maybe_start_random_turn(self, now):
        if now >= self.random_cruise_until and now >= self.random_turn_until:
            self.turn_direction = random.choice([-1.0, 1.0])
            self.random_turn_until = now + self.sample_turn_duration()
            self.random_cruise_until = self.random_turn_until + self.sample_cruise_duration()

    def compute_command(self, now):
        if self.last_scan is None or now - self.last_scan_time > self.scan_timeout_sec:
            self.set_mode("waiting_for_scan")
            return self.build_command(0.0, 0.0)

        scan = self.last_scan
        front_min = self.sector_min(scan, -self.front_half_angle_deg, self.front_half_angle_deg)
        left_min = self.sector_min(scan, self.front_half_angle_deg, self.side_sector_deg)
        right_min = self.sector_min(scan, -self.side_sector_deg, -self.front_half_angle_deg)

        if front_min < self.emergency_clearance:
            self.turn_direction = self.select_turn_direction(left_min, right_min)
            self.random_turn_until = now + self.sample_turn_duration()
            self.random_cruise_until = self.random_turn_until + self.sample_cruise_duration()
            self.set_mode("escape_turn")
            return self.build_command(0.0, self.turn_direction * self.escape_angular_speed)

        if front_min < self.forward_clearance or left_min < self.side_clearance or right_min < self.side_clearance:
            self.turn_direction = self.select_turn_direction(left_min, right_min)
            self.random_turn_until = now + self.sample_turn_duration()
            self.random_cruise_until = self.random_turn_until + self.sample_cruise_duration()
            self.set_mode("avoid_turn")
            return self.build_command(0.02, self.turn_direction * self.angular_speed)

        self.maybe_start_random_turn(now)
        if now < self.random_turn_until:
            self.set_mode("random_turn")
            return self.build_command(0.04, self.turn_direction * (0.7 * self.angular_speed))

        self.set_mode("cruise")
        steer_bias = max(min((right_min - left_min) * 0.45, 0.20), -0.20)
        return self.build_command(self.linear_speed, steer_bias)

    def run(self):
        rospy.loginfo("wander_obstacle_avoidance is running.")
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            now = time.time()
            cmd = self.compute_command(now)
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def shutdown(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("wander_obstacle_avoidance")
    WanderObstacleAvoidance().run()
