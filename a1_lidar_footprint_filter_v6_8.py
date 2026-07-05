#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A1 LiDAR body-footprint safety filter v6.8 for ROS Melodic / Python2.

Input : /tmp/a1_follow_cmd_raw   (enable vx vy wz stamp)
Output: /tmp/a1_follow_cmd       (filtered command read by a1_high_follow_driver)
Debug : /tmp/a1_body_footprint_debug and /tmp/a1_body_safety_debug
Backup: /tmp/a1_obstacle_front_m (same 0.70 m hard-stop reference as the known v2 setup)

Design reset from field results:
  - v2 visual-follow parameters worked best. Do not over-filter them.
  - Do not inject lateral bias by default; it caused left drift.
  - Use LiDAR only as a hard/soft safety layer.
  - Front hard stop is 0.70 m, matching the known clean behavior.
  - A1 is not a point: body size, LiDAR offset, and walking envelope are used
    for side/rear protection, but side/rear logic only limits motion when really close.

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
FRONT_PATH = "/tmp/a1_obstacle_front_m"


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
    vy = clamp(float(vy), -0.05, 0.05)
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


def write_front(front):
    try:
        tmp = FRONT_PATH + ".tmp"
        with open(tmp, "w") as f:
            f.write("%.4f\n" % front)
        os.rename(tmp, FRONT_PATH)
    except Exception:
        pass


def footprint_params():
    body_len = rospy.get_param("~body_length_m", 0.50)
    body_w = rospy.get_param("~body_width_m", 0.30)
    lidar_x = rospy.get_param("~lidar_x_from_body_center_m", 0.20)
    dyn_front = rospy.get_param("~dynamic_front_extra_m", 0.12)
    dyn_rear = rospy.get_param("~dynamic_rear_extra_m", 0.22)
    dyn_side = rospy.get_param("~dynamic_side_extra_m", 0.08)
    margin = rospy.get_param("~safety_margin_m", 0.05)

    front_x = body_len * 0.5 - lidar_x + dyn_front + margin
    rear_x = -body_len * 0.5 - lidar_x - dyn_rear - margin
    half_w = body_w * 0.5 + dyn_side + margin
    return rear_x, front_x, half_w


def front_sector_min(points, half_deg, max_r=1.35, q=0.03):
    # Narrow-to-medium sector; this preserves the known 0.70 m stop behavior without
    # treating every side wall as a front obstacle.
    half = math.radians(half_deg)
    vals = []
    for x, y, r, a in points:
        if x > 0.02 and r <= max_r and abs(a) <= half:
            vals.append(r)
    return percentile(vals, q, 999.0)


def front_corridor_min(points, half_w, max_x, q=0.03):
    # Obstacle inside the robot-width forward corridor. Return forward x-distance.
    vals = []
    for x, y, r, a in points:
        if 0.02 <= x <= max_x and abs(y) <= half_w:
            vals.append(x)
    return percentile(vals, q, 999.0)


def side_clearances(points, rear_x, front_x, half_w):
    left_all, right_all, left_rear, right_rear = [], [], [], []
    for x, y, r, a in points:
        if rear_x <= x <= front_x:
            if y > half_w:
                c = y - half_w
                left_all.append(c)
                if x < -0.10:
                    left_rear.append(c)
            elif y < -half_w:
                c = -half_w - y
                right_all.append(c)
                if x < -0.10:
                    right_rear.append(c)
    q = 0.03
    return {
        "left": percentile(left_all, q),
        "right": percentile(right_all, q),
        "left_rear": percentile(left_rear, q),
        "right_rear": percentile(right_rear, q),
    }


def scale_between(x, hard, soft):
    if x <= hard:
        return 0.0
    if x >= soft:
        return 1.0
    return clamp((x - hard) / max(1e-6, soft - hard), 0.0, 1.0)


def filter_cmd(en, vx, vy, wz, points):
    rear_x, front_x, half_w = footprint_params()

    # Reset to v2-compatible safety: 0.70 m hard stop.
    front_stop = rospy.get_param("~front_stop_m", 0.70)
    front_turn = rospy.get_param("~front_turn_only_m", 0.85)
    front_slow = rospy.get_param("~front_slow_m", 1.05)
    front_sector_deg = rospy.get_param("~front_sector_half_deg", 35.0)

    side_hard = rospy.get_param("~side_hard_clearance_m", 0.03)
    side_soft = rospy.get_param("~side_soft_clearance_m", 0.10)
    rear_hard = rospy.get_param("~rear_hard_clearance_m", 0.04)
    rear_soft = rospy.get_param("~rear_soft_clearance_m", 0.12)

    max_vx = rospy.get_param("~max_vx", 0.16)
    max_wz = rospy.get_param("~max_wz", 0.45)
    max_wz_near = rospy.get_param("~max_wz_near", 0.25)
    max_vx_side_soft = rospy.get_param("~max_vx_side_soft", 0.10)
    max_vx_side_hard = rospy.get_param("~max_vx_side_hard", 0.05)

    # Zero by default. Do not add lateral correction unless explicitly calibrated.
    vy_bias = rospy.get_param("~vy_bias", 0.0)
    vy_sign = rospy.get_param("~vy_sign", 1.0)
    wz_bias = rospy.get_param("~wz_bias", 0.0)
    wz_sign = rospy.get_param("~wz_sign", 1.0)
    rear_swing_protect = rospy.get_param("~rear_swing_protect", True)

    raw_vx, raw_vy, raw_wz = vx, vy, wz
    reason = []

    vx = clamp(vx, 0.0, max_vx)
    vy = clamp(vy + vy_sign * vy_bias, -0.05, 0.05)
    wz = clamp(wz + wz_sign * wz_bias, -max_wz, max_wz)

    sector = front_sector_min(points, front_sector_deg, max_r=front_slow + 0.30)
    corridor = front_corridor_min(points, half_w + 0.05, front_slow + 0.30)
    front = min(sector, corridor)
    write_front(front)

    cs = side_clearances(points, rear_x, front_x, half_w)
    left = cs["left"]
    right = cs["right"]
    left_rear = cs["left_rear"]
    right_rear = cs["right_rear"]

    if front < front_stop:
        en = 0
        vx = vy = wz = 0.0
        reason.append("front_0p70_hard_stop")
    elif front < front_turn:
        vx = 0.0
        # do not translate sideways near a front obstacle; rotate only to reacquire/avoid
        vy = 0.0
        wz = clamp(wz, -max_wz_near, max_wz_near)
        reason.append("front_turn_only")
    elif front < front_slow and vx > 0.0:
        vx *= scale_between(front, front_turn, front_slow)
        reason.append("front_slow")

    if en:
        # Side protection should not create left drift. It only blocks dangerous motion and limits vx.
        if left < side_soft and vy > 0.0:
            vy = 0.0
            reason.append("block_left_vy")
        if right < side_soft and vy < 0.0:
            vy = 0.0
            reason.append("block_right_vy")

        min_side = min(left, right)
        if min_side < side_hard and vx > max_vx_side_hard:
            vx = max_vx_side_hard
            reason.append("side_hard_slow_vx")
        elif min_side < side_soft and vx > max_vx_side_soft:
            vx = max_vx_side_soft
            reason.append("side_soft_slow_vx")

        # Rear foot swing protection. This limits yaw only very close to rear side obstacles.
        if rear_swing_protect:
            rear_corner_vy = vy + wz * rear_x
            # At rear_x < 0, -wz moves rear toward robot-left, +wz moves rear toward robot-right.
            if left_rear < rear_hard and rear_corner_vy > 0.0:
                if wz < 0.0:
                    wz = 0.0
                if vy > 0.0:
                    vy = 0.0
                reason.append("left_rear_block")
            elif left_rear < rear_soft and rear_corner_vy > 0.0 and wz < 0.0:
                wz *= scale_between(left_rear, rear_hard, rear_soft)
                reason.append("left_rear_slow")

            if right_rear < rear_hard and rear_corner_vy < 0.0:
                if wz > 0.0:
                    wz = 0.0
                if vy < 0.0:
                    vy = 0.0
                reason.append("right_rear_block")
            elif right_rear < rear_soft and rear_corner_vy < 0.0 and wz > 0.0:
                wz *= scale_between(right_rear, rear_hard, rear_soft)
                reason.append("right_rear_slow")

    vx = clamp(vx, 0.0, max_vx)
    vy = clamp(vy, -0.05, 0.05)
    wz = clamp(wz, -max_wz, max_wz)

    if not reason:
        reason = ["pass_v2"]

    dbg = ("reason=%s raw=%d %.3f %.3f %.3f out=%d %.3f %.3f %.3f "
           "front=%.3f sector=%.3f corridor=%.3f left=%.3f right=%.3f "
           "lr=%.3f rr=%.3f footprint_x=[%.2f,%.2f] half_w=%.2f") % (
        "+".join(reason), en, raw_vx, raw_vy, raw_wz, en, vx, vy, wz,
        front, sector, corridor, left, right, left_rear, right_rear,
        rear_x, front_x, half_w)
    return en, vx, vy, wz, dbg


def main():
    rospy.init_node("a1_lidar_footprint_filter_v6_8")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.loginfo("a1_lidar_footprint_filter_v6_8 started scan=%s raw=%s out=%s", scan_topic, RAW_CMD_PATH, OUT_CMD_PATH)

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
            # LiDAR stream is stale. Do NOT write front=0.0 because the Unitree
            # driver interprets that as a real front obstacle and hides the true
            # cause. Stop final motion, but mark front as unknown/far.
            write_front(999.0)
            write_cmd(0, 0.0, 0.0, 0.0)
            dbg = "reason=no_recent_scan_stop raw=%d %.3f %.3f %.3f raw_age=%.3f scan_age=%.3f" % (en, vx, vy, wz, raw_age, scan_age)
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
