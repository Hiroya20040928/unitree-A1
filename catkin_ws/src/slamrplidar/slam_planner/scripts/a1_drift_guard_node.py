#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def yaw_from_q(q):
    return tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

class A1DriftGuard:
    def __init__(self):
        self.max_vx = rospy.get_param("~max_vx", 0.85)
        self.max_wz = rospy.get_param("~max_wz", 0.24)

        # plannerの微小前進を，A1が歩容に乗る速度へ持ち上げる
        self.min_useful_vx = rospy.get_param("~min_useful_vx", 0.55)
        self.enable_speed_lift = rospy.get_param("~enable_speed_lift", True)

        # 急加速抑制
        self.accel_vx = rospy.get_param("~accel_vx", 0.28)
        self.decel_vx = rospy.get_param("~decel_vx", 0.90)
        self.accel_wz = rospy.get_param("~accel_wz", 0.26)

        # 直進時のみ横ドリフト補正
        self.straight_wz_threshold = rospy.get_param("~straight_wz_threshold", 0.06)
        self.k_lat = rospy.get_param("~k_lat", 0.75)
        self.k_yaw = rospy.get_param("~k_yaw", 0.45)
        self.max_drift_corr = rospy.get_param("~max_drift_corr", 0.12)

        # 弱いfeed-forward．直進時のみ
        self.v_ref = rospy.get_param("~v_ref", 1.0)
        self.k_ff = rospy.get_param("~k_ff", 0.015)
        self.max_ff = rospy.get_param("~max_ff", 0.02)

        # ゴール近傍停止．大きすぎると近場goalで動かないため小さめ
        self.goal_stop_radius = rospy.get_param("~goal_stop_radius", 0.45)
        self.goal_latch_radius = rospy.get_param("~goal_latch_radius", 0.38)

        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.35)

        self.listener = tf.TransformListener()

        self.raw = Twist()
        self.last_raw_time = rospy.Time(0)

        self.odom_ok = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.out_vx = 0.0
        self.out_wz = 0.0
        self.last_update = rospy.Time.now()

        self.anchor_active = False
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0

        self.goal_enabled = False
        self.goal_reached_latched = False
        self.goal_odom_x = 0.0
        self.goal_odom_y = 0.0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub_cmd = rospy.Subscriber("/cmd_vel_raw", Twist, self.cb_cmd, queue_size=10)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.cb_odom, queue_size=20)
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=3)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)

    def cb_cmd(self, msg):
        self.raw = msg
        self.last_raw_time = rospy.Time.now()

    def cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_q(msg.pose.pose.orientation)
        self.odom_ok = True

    def cb_goal(self, msg):
        self.goal_reached_latched = False
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose
            self.listener.waitForTransform("odom", ps.header.frame_id, ps.header.stamp, rospy.Duration(0.25))
            p = self.listener.transformPose("odom", ps)
            self.goal_odom_x = p.pose.position.x
            self.goal_odom_y = p.pose.position.y
            self.goal_enabled = True
        except Exception:
            self.goal_odom_x = msg.pose.position.x
            self.goal_odom_y = msg.pose.position.y
            self.goal_enabled = True

    def rate_limit(self, cur, target, rate, dt):
        step = rate * dt
        return clamp(target, cur - step, cur + step)

    def publish_zero(self):
        self.out_vx = 0.0
        self.out_wz = 0.0
        self.anchor_active = False
        self.pub.publish(Twist())

    def goal_distance(self):
        if not self.goal_enabled or not self.odom_ok:
            return None
        dx = self.goal_odom_x - self.x
        dy = self.goal_odom_y - self.y
        return math.sqrt(dx*dx + dy*dy)

    def update(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_update).to_sec()
        self.last_update = now
        if dt <= 0.0 or dt > 0.3:
            dt = 0.05

        if self.last_raw_time == rospy.Time(0) or (now - self.last_raw_time).to_sec() > self.cmd_timeout:
            self.publish_zero()
            return

        d_goal = self.goal_distance()
        if d_goal is not None:
            if d_goal < self.goal_latch_radius:
                self.goal_reached_latched = True
            if self.goal_reached_latched or d_goal < self.goal_stop_radius:
                self.publish_zero()
                return

        raw_vx = clamp(self.raw.linear.x, -self.max_vx, self.max_vx)
        raw_wz = clamp(self.raw.angular.z, -self.max_wz, self.max_wz)

        target_vx = raw_vx

        # 前進命令が少しでも出たら，A1の歩容が成立する速度まで持ち上げる
        # ただしplannerが停止・旋回だけを要求している時は持ち上げない
        if self.enable_speed_lift and raw_vx > 0.03:
            target_vx = max(raw_vx, self.min_useful_vx)

        straight_mode = (
            self.odom_ok and
            target_vx > 0.20 and
            abs(raw_wz) < self.straight_wz_threshold
        )

        drift_corr = 0.0

        if straight_mode:
            if not self.anchor_active:
                self.anchor_active = True
                self.anchor_x = self.x
                self.anchor_y = self.y
                self.anchor_yaw = self.yaw

            dx = self.x - self.anchor_x
            dy = self.y - self.anchor_y
            e_lat = -math.sin(self.anchor_yaw) * dx + math.cos(self.anchor_yaw) * dy
            e_yaw = wrap_pi(self.yaw - self.anchor_yaw)

            fb = -self.k_lat * e_lat - self.k_yaw * e_yaw
            fb = clamp(fb, -self.max_drift_corr, self.max_drift_corr)

            ff = 0.0
            if target_vx < self.v_ref:
                ff = -self.k_ff * (self.v_ref - target_vx)
                ff = clamp(ff, -self.max_ff, self.max_ff)

            drift_corr = clamp(fb + ff, -self.max_drift_corr, self.max_drift_corr)
        else:
            self.anchor_active = False

        target_wz = clamp(raw_wz + drift_corr, -self.max_wz, self.max_wz)

        vx_rate = self.accel_vx if abs(target_vx) > abs(self.out_vx) else self.decel_vx
        self.out_vx = self.rate_limit(self.out_vx, target_vx, vx_rate, dt)
        self.out_wz = self.rate_limit(self.out_wz, target_wz, self.accel_wz, dt)

        out = Twist()
        out.linear.x = self.out_vx
        out.angular.z = self.out_wz
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("a1_drift_guard_node")
    A1DriftGuard()
    rospy.spin()
