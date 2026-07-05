#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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

class A1LateralGuard:
    def __init__(self):
        self.max_vx = rospy.get_param("~max_vx", 0.75)
        self.max_vy = rospy.get_param("~max_vy", 0.22)
        self.max_wz = rospy.get_param("~max_wz", 0.24)

        self.accel_vx = rospy.get_param("~accel_vx", 0.30)
        self.accel_vy = rospy.get_param("~accel_vy", 0.35)
        self.accel_wz = rospy.get_param("~accel_wz", 0.25)
        self.decel_vx = rospy.get_param("~decel_vx", 0.90)

        self.min_walk_vx = rospy.get_param("~min_walk_vx", 0.45)
        self.speed_lift_threshold = rospy.get_param("~speed_lift_threshold", 0.05)

        self.k_lat = rospy.get_param("~k_lat", 0.85)
        self.max_lat_corr = rospy.get_param("~max_lat_corr", 0.20)
        self.lat_sign = rospy.get_param("~lat_sign", -1.0)

        self.straight_wz_threshold = rospy.get_param("~straight_wz_threshold", 0.06)
        self.enable_lateral_correction = rospy.get_param("~enable_lateral_correction", True)

        self.front_stop_dist = rospy.get_param("~front_stop_dist", 0.65)
        self.front_slow_dist = rospy.get_param("~front_slow_dist", 1.00)
        self.front_sector_deg = rospy.get_param("~front_sector_deg", 35.0)

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

        self.front_min = float("inf")
        self.scan_ok = False
        self.last_scan_time = rospy.Time(0)

        self.out_vx = 0.0
        self.out_vy = 0.0
        self.out_wz = 0.0
        self.last_update = rospy.Time.now()

        self.anchor_active = False
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0

        self.goal_enabled = False
        self.goal_latched = False
        self.goal_odom_x = 0.0
        self.goal_odom_y = 0.0

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.dbg = rospy.Publisher("/a1_nav_debug", String, queue_size=20)

        rospy.Subscriber("/cmd_vel_nav", Twist, self.cb_cmd, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.cb_odom, queue_size=20)
        rospy.Subscriber("/scan", LaserScan, self.cb_scan, queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.cb_goal, queue_size=3)

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
        self.goal_latched = False
        try:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose
            self.listener.waitForTransform("odom", ps.header.frame_id, ps.header.stamp, rospy.Duration(0.25))
            p = self.listener.transformPose("odom", ps)
            self.goal_odom_x = p.pose.position.x
            self.goal_odom_y = p.pose.position.y
        except Exception:
            self.goal_odom_x = msg.pose.position.x
            self.goal_odom_y = msg.pose.position.y
        self.goal_enabled = True

    def cb_scan(self, msg):
        front = float("inf")
        sector = math.radians(self.front_sector_deg)

        angle = msg.angle_min
        for r in msg.ranges:
            if (not math.isnan(r) and not math.isinf(r)) and msg.range_min <= r <= msg.range_max:
                a = wrap_pi(angle)
                if abs(a) <= sector:
                    front = min(front, r)
            angle += msg.angle_increment

        self.front_min = front
        self.scan_ok = True
        self.last_scan_time = rospy.Time.now()

    def rate_limit(self, cur, target, rate, dt):
        step = rate * dt
        return clamp(target, cur - step, cur + step)

    def publish_zero(self, reason):
        self.out_vx = 0.0
        self.out_vy = 0.0
        self.out_wz = 0.0
        self.anchor_active = False
        self.pub.publish(Twist())
        self.dbg.publish(String("STOP reason=%s front_min=%.3f x=%.3f y=%.3f yaw=%.3f" %
                                (reason, self.front_min, self.x, self.y, self.yaw)))

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
        if dt <= 0.0 or dt > 0.30:
            dt = 0.05

        if self.last_raw_time == rospy.Time(0) or (now - self.last_raw_time).to_sec() > self.cmd_timeout:
            self.publish_zero("cmd_timeout")
            return

        d_goal = self.goal_distance()
        if d_goal is not None:
            if d_goal < self.goal_latch_radius:
                self.goal_latched = True
            if self.goal_latched or d_goal < self.goal_stop_radius:
                self.publish_zero("goal_reached")
                return

        scan_age = (now - self.last_scan_time).to_sec() if self.scan_ok else 999.0
        if scan_age > 0.50:
            self.publish_zero("scan_timeout")
            return

        raw_vx = clamp(self.raw.linear.x, -self.max_vx, self.max_vx)
        raw_vy = clamp(self.raw.linear.y, -self.max_vy, self.max_vy)
        raw_wz = clamp(self.raw.angular.z, -self.max_wz, self.max_wz)

        # 前方障害物がscanに見えているなら，ソフト側で確実に止める
        if raw_vx > 0.0 and self.front_min < self.front_stop_dist:
            self.publish_zero("front_obstacle")
            return

        target_vx = raw_vx
        if raw_vx > self.speed_lift_threshold:
            target_vx = max(raw_vx, self.min_walk_vx)

        # front_slow_dist内では速度上限を落とす
        if self.front_min < self.front_slow_dist and target_vx > 0.0:
            scale = clamp((self.front_min - self.front_stop_dist) /
                          (self.front_slow_dist - self.front_stop_dist), 0.0, 1.0)
            target_vx = min(target_vx, 0.20 + 0.35 * scale)

        target_vy = raw_vy
        straight_mode = (
            self.enable_lateral_correction and
            self.odom_ok and
            target_vx > 0.20 and
            abs(raw_wz) < self.straight_wz_threshold
        )

        e_lat = 0.0
        lat_corr = 0.0

        if straight_mode:
            if not self.anchor_active:
                self.anchor_active = True
                self.anchor_x = self.x
                self.anchor_y = self.y
                self.anchor_yaw = self.yaw

            dx = self.x - self.anchor_x
            dy = self.y - self.anchor_y

            # anchor進行方向に対する横偏差．左ずれを正とする
            e_lat = -math.sin(self.anchor_yaw) * dx + math.cos(self.anchor_yaw) * dy

            # 左へ流れたら右向きlinear.yを入れる．符号はlat_signで反転可能
            lat_corr = self.lat_sign * self.k_lat * e_lat
            lat_corr = clamp(lat_corr, -self.max_lat_corr, self.max_lat_corr)
            target_vy = clamp(raw_vy + lat_corr, -self.max_vy, self.max_vy)
        else:
            self.anchor_active = False

        target_wz = raw_wz

        vx_rate = self.accel_vx if abs(target_vx) > abs(self.out_vx) else self.decel_vx
        self.out_vx = self.rate_limit(self.out_vx, target_vx, vx_rate, dt)
        self.out_vy = self.rate_limit(self.out_vy, target_vy, self.accel_vy, dt)
        self.out_wz = self.rate_limit(self.out_wz, target_wz, self.accel_wz, dt)

        out = Twist()
        out.linear.x = self.out_vx
        out.linear.y = self.out_vy
        out.angular.z = self.out_wz
        self.pub.publish(out)

        self.dbg.publish(String(
            "raw_vx=%.3f raw_vy=%.3f raw_wz=%.3f out_vx=%.3f out_vy=%.3f out_wz=%.3f e_lat=%.3f lat_corr=%.3f front_min=%.3f goal_dist=%s straight=%d" %
            (raw_vx, raw_vy, raw_wz, self.out_vx, self.out_vy, self.out_wz,
             e_lat, lat_corr, self.front_min,
             ("None" if d_goal is None else "%.3f" % d_goal),
             1 if straight_mode else 0)
        ))

if __name__ == "__main__":
    rospy.init_node("a1_lateral_guard_node")
    A1LateralGuard()
    rospy.spin()
