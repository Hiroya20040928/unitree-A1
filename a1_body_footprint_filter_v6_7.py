#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A1 body-footprint safety filter v6.7 for ROS Melodic / Python2.

Input : /tmp/a1_follow_cmd_raw   (enable vx vy wz stamp)
Output: /tmp/a1_follow_cmd       (filtered command read by a1_high_follow_driver)
Debug : /tmp/a1_body_footprint_debug

Purpose:
  - Do not treat the A1 as a point.
  - Treat the A1 as an inflated walking footprint, including body size,
    LiDAR mounting offset, dynamic foot swing, and safety margin.
  - Prioritize: front hard stop > front turn-only > side/rear swing limiting.

Coordinate convention in /scan laser frame:
  x: forward from LiDAR
  y: robot-left from LiDAR
  +vx: forward, +vy: robot-left, +wz: left yaw.
"""
from __future__ import print_function

import math
import os
import time
import threading

import rospy
from sensor_msgs.msg import LaserScan

RAW_CMD_PATH = "/tmp/a1_follow_cmd_raw"
OUT_CMD_PATH = "/tmp/a1_follow_cmd"
DEBUG_PATH = "/tmp/a1_body_footprint_debug"
LEGACY_DEBUG_PATH = "/tmp/a1_body_safety_debug"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def isfinite(x):
    try:
        return math.isfinite(x)
    except AttributeError:
        return not (math.isinf(x) or math.isnan(x))


def percentile(vals, q, default=999.0):
    if not vals:
        return default
    vals = sorted(vals)
    i = int(clamp(q, 0.0, 1.0) * (len(vals) - 1))
    return vals[i]


class Shared(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.points = []
        self.stamp = 0.0


S = Shared()


def scan_cb(msg):
    pts = []
    a = msg.angle_min
    inc = msg.angle_increment
    for r in msg.ranges:
        try:
            rf = float(r)
        except Exception:
            a += inc
            continue
        if isfinite(rf) and msg.range_min <= rf <= msg.range_max:
            pts.append((rf * math.cos(a), rf * math.sin(a), rf, a))
        a += inc
    with S.lock:
        S.points = pts
        S.stamp = time.time()


def read_raw_cmd():
    try:
        with open(RAW_CMD_PATH, "r") as f:
            parts = f.read().strip().split()
        if len(parts) < 5:
            return 0, 0.0, 0.0, 0.0, 0.0
        return int(float(parts[0])), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
    except Exception:
        return 0, 0.0, 0.0, 0.0, 0.0


def write_cmd(enable, vx, vy, wz):
    vx = clamp(float(vx), -0.03, 0.16)
    vy = clamp(float(vy), -0.07, 0.07)
    wz = clamp(float(wz), -0.45, 0.45)
    tmp = OUT_CMD_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (1 if enable else 0, vx, vy, wz, time.time()))
    try:
        os.rename(tmp, OUT_CMD_PATH)
    except Exception:
        with open(OUT_CMD_PATH, "w") as f:
            f.write("%d %.5f %.5f %.5f %.6f\n" % (1 if enable else 0, vx, vy, wz, time.time()))


def write_debug(s):
    for path in (DEBUG_PATH, LEGACY_DEBUG_PATH):
        try:
            with open(path, "w") as f:
                f.write(s + "\n")
        except Exception:
            pass


def footprint_params():
    body_len = rospy.get_param("~body_length_m", 0.50)
    body_w = rospy.get_param("~body_width_m", 0.30)
    lidar_x = rospy.get_param("~lidar_x_from_body_center_m", 0.20)
    dyn_front = rospy.get_param("~dynamic_front_extra_m", 0.16)
    dyn_rear = rospy.get_param("~dynamic_rear_extra_m", 0.26)
    dyn_side = rospy.get_param("~dynamic_side_extra_m", 0.12)
    margin = rospy.get_param("~safety_margin_m", 0.07)

    # LiDAR-frame inflated walking envelope.
    front_x = body_len * 0.5 - lidar_x + dyn_front + margin
    rear_x = -body_len * 0.5 - lidar_x - dyn_rear - margin
    half_w = body_w * 0.5 + dyn_side + margin
    return rear_x, front_x, half_w


def sector_min(points, half_deg, max_r=2.0, q=0.05):
    half = math.radians(half_deg)
    vals = []
    for x, y, r, a in points:
        if x > 0.02 and r <= max_r and abs(a) <= half:
            vals.append(r)
    return percentile(vals, q, 999.0)


def front_rect_min(points, half_w, max_x, q=0.05):
    vals = []
    for x, y, r, a in points:
        if 0.02 <= x <= max_x and abs(y) <= half_w:
            vals.append(x)
    return percentile(vals, q, 999.0)


def side_and_rear_clearances(points, rear_x, front_x, half_w):
    # Clearance from inflated footprint boundary. Positive is free space outside footprint.
    left_all = []
    right_all = []
    left_rear = []
    right_rear = []
    left_front = []
    right_front = []

    for x, y, r, a in points:
        if rear_x <= x <= front_x:
            if y > half_w:
                c = y - half_w
                left_all.append(c)
                if x < -0.10:
                    left_rear.append(c)
                if x >= -0.10:
                    left_front.append(c)
            elif y < -half_w:
                c = -half_w - y
                right_all.append(c)
                if x < -0.10:
                    right_rear.append(c)
                if x >= -0.10:
                    right_front.append(c)

    q = 0.05
    return {
        "left": percentile(left_all, q),
        "right": percentile(right_all, q),
        "left_rear": percentile(left_rear, q),
        "right_rear": percentile(right_rear, q),
        "left_front": percentile(left_front, q),
        "right_front": percentile(right_front, q),
    }


def scale_between(x, hard, soft):
    if x <= hard:
        return 0.0
    if x >= soft:
        return 1.0
    return clamp((x - hard) / max(1e-6, soft - hard), 0.0, 1.0)


def filter_cmd(en, vx, vy, wz, points):
    rear_x, front_x, half_w = footprint_params()

    # Front safety. Use both a wide sector and the actual body-width corridor.
    front_stop = rospy.get_param("~front_stop_m", 0.85)
    front_turn = rospy.get_param("~front_turn_only_m", 1.15)
    front_slow = rospy.get_param("~front_slow_m", 1.40)
    front_sector_deg = rospy.get_param("~front_sector_half_deg", 60.0)

    # Side/rear safety. These are clearance beyond the inflated footprint.
    side_hard = rospy.get_param("~side_hard_clearance_m", 0.06)
    side_soft = rospy.get_param("~side_soft_clearance_m", 0.18)
    desired_side = rospy.get_param("~desired_side_clearance_m", 0.28)
    rear_hard = rospy.get_param("~rear_hard_clearance_m", 0.08)
    rear_soft = rospy.get_param("~rear_soft_clearance_m", 0.22)

    k_side_vy = rospy.get_param("~k_side_vy", 0.050)
    max_side_vy = rospy.get_param("~max_side_vy", 0.035)
    max_vx = rospy.get_param("~max_vx", 0.10)
    max_wz = rospy.get_param("~max_wz", 0.38)
    max_wz_near = rospy.get_param("~max_wz_near", 0.18)
    max_vx_side_soft = rospy.get_param("~max_vx_side_soft", 0.055)
    max_vx_side_hard = rospy.get_param("~max_vx_side_hard", 0.030)

    vy_bias = rospy.get_param("~vy_bias", 0.0)
    vy_sign = rospy.get_param("~vy_sign", 1.0)
    wz_bias = rospy.get_param("~wz_bias", 0.0)
    wz_sign = rospy.get_param("~wz_sign", 1.0)
    rear_swing_protect = rospy.get_param("~rear_swing_protect", True)

    raw_vx, raw_vy, raw_wz = vx, vy, wz
    reason = []

    vx = clamp(vx, 0.0, max_vx)
    vy = clamp(vy + vy_sign * vy_bias, -0.07, 0.07)
    wz = clamp(wz + wz_sign * wz_bias, -max_wz, max_wz)

    front_sector = sector_min(points, front_sector_deg, max_r=front_slow + 0.35)
    front_rect = front_rect_min(points, half_w, front_slow + 0.35)
    front = min(front_sector, front_rect)

    cs = side_and_rear_clearances(points, rear_x, front_x, half_w)
    left = min(cs["left"], cs["left_front"], cs["left_rear"])
    right = min(cs["right"], cs["right_front"], cs["right_rear"])
    left_rear = cs["left_rear"]
    right_rear = cs["right_rear"]

    # 1) Front hard stop / turn only / slow.
    if front < front_stop:
        en = 0
        vx = 0.0
        vy = 0.0
        wz = 0.0
        reason.append("front_hard_stop")
    elif front < front_turn:
        vx = 0.0
        vy = 0.0
        wz = clamp(wz, -max_wz_near, max_wz_near)
        reason.append("front_turn_only")
    elif front < front_slow and vx > 0.0:
        vx *= scale_between(front, front_turn, front_slow)
        reason.append("front_slow")

    # 2) Bias away from close sides, but do not let side correction dominate the visual yaw.
    if en:
        side_corr = 0.0
        if left < desired_side:
            side_corr -= k_side_vy * (desired_side - left)
            reason.append("left_clear")
        if right < desired_side:
            side_corr += k_side_vy * (desired_side - right)
            reason.append("right_clear")
        side_corr = clamp(side_corr, -max_side_vy, max_side_vy)
        vy += side_corr

        # Block commands into a close side.
        if left < side_soft and vy > 0.0:
            vy = min(vy, 0.0)
            reason.append("block_left_vy")
        if right < side_soft and vy < 0.0:
            vy = max(vy, 0.0)
            reason.append("block_right_vy")

        min_side = min(left, right)
        if min_side < side_hard and vx > max_vx_side_hard:
            vx = max_vx_side_hard
            reason.append("side_hard_slow_vx")
        elif min_side < side_soft and vx > max_vx_side_soft:
            vx = max_vx_side_soft
            reason.append("side_soft_slow_vx")

        # 3) Rear foot swing protection. At rear_x < 0, +wz swings rear to the right,
        # -wz swings rear to the left. Limit exactly those rotations near rear-side obstacles.
        if rear_swing_protect:
            rear_corner_vy = vy + wz * rear_x
            if left_rear < rear_hard and rear_corner_vy > 0.0:
                if vy > 0.0:
                    vy = 0.0
                if wz < 0.0:
                    wz = 0.0
                reason.append("left_rear_block")
            elif left_rear < rear_soft and rear_corner_vy > 0.0:
                if wz < 0.0:
                    wz *= scale_between(left_rear, rear_hard, rear_soft)
                reason.append("left_rear_slow")

            if right_rear < rear_hard and rear_corner_vy < 0.0:
                if vy < 0.0:
                    vy = 0.0
                if wz > 0.0:
                    wz = 0.0
                reason.append("right_rear_block")
            elif right_rear < rear_soft and rear_corner_vy < 0.0:
                if wz > 0.0:
                    wz *= scale_between(right_rear, rear_hard, rear_soft)
                reason.append("right_rear_slow")

    vx = clamp(vx, 0.0, max_vx)
    vy = clamp(vy, -0.07, 0.07)
    wz = clamp(wz, -max_wz, max_wz)

    if not reason:
        reason = ["pass"]

    dbg = ("reason=%s raw=%d %.3f %.3f %.3f out=%d %.3f %.3f %.3f "
           "front=%.3f sector=%.3f rect=%.3f left=%.3f right=%.3f "
           "lr=%.3f rr=%.3f footprint_x=[%.2f,%.2f] half_w=%.2f") % (
        "+".join(reason), en, raw_vx, raw_vy, raw_wz, en, vx, vy, wz,
        front, front_sector, front_rect, left, right, left_rear, right_rear,
        rear_x, front_x, half_w)
    return en, vx, vy, wz, dbg


def main():
    rospy.init_node("a1_body_footprint_filter_v6_6")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.loginfo("a1_body_footprint_filter_v6_6 started scan=%s raw=%s out=%s", scan_topic, RAW_CMD_PATH, OUT_CMD_PATH)

    rate = rospy.Rate(rospy.get_param("~rate", 35.0))
    raw_timeout = rospy.get_param("~raw_timeout", 0.50)
    scan_timeout = rospy.get_param("~scan_timeout", 0.70)
    monitor_only = rospy.get_param("~monitor_only", False)
    last_log = 0.0

    while not rospy.is_shutdown():
        now = time.time()
        en, vx, vy, wz, st = read_raw_cmd()
        raw_age = now - st if st > 0.0 else 999.0
        with S.lock:
            points = list(S.points)
            scan_age = now - S.stamp if S.stamp > 0.0 else 999.0

        if en == 0 or st <= 0.0 or raw_age > raw_timeout:
            write_cmd(0, 0.0, 0.0, 0.0)
            dbg = "reason=disabled_or_raw_timeout raw_en=%d raw_age=%.3f scan_age=%.3f" % (en, raw_age, scan_age)
        elif not points or scan_age > scan_timeout:
            # If LiDAR is unavailable, do not advance blindly.
            write_cmd(1, 0.0, 0.0, clamp(wz, -0.12, 0.12))
            dbg = "reason=no_recent_scan_turn_only raw=%d %.3f %.3f %.3f raw_age=%.3f scan_age=%.3f" % (en, vx, vy, wz, raw_age, scan_age)
        else:
            en2, vx2, vy2, wz2, dbg = filter_cmd(en, vx, vy, wz, points)
            if monitor_only:
                write_cmd(en, vx, vy, wz)
                dbg = "monitor_only+" + dbg
            else:
                write_cmd(en2, vx2, vy2, wz2)

        write_debug(dbg)
        if now - last_log > 0.5:
            last_log = now
            rospy.loginfo(dbg)
        rate.sleep()


if __name__ == "__main__":
    main()
