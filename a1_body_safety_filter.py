#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A1 body/footprint safety filter for ROS Melodic (Python2 compatible).

Reads raw velocity commands from /tmp/a1_follow_cmd_raw and writes filtered commands to
/tmp/a1_follow_cmd for the existing Unitree high-level driver.

Purpose:
- Treat A1 as a finite-width / finite-length body, not as a point.
- Keep side/rear-leg clearance from walls and furniture.
- Preserve the already-confirmed original front obstacle stop behavior.
- Add a small constant lateral bias to counter observed natural left drift.

Coordinate convention used here:
- +vx: forward
- +vy: robot-left, -vy: robot-right
- +wz: left yaw, -wz: right yaw
If your machine behaves with opposite side/yaw signs, change _vy_sign or _wz_sign at launch.
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
DEBUG_PATH = "/tmp/a1_body_safety_debug"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Shared(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.scan = None
        self.scan_stamp = 0.0
        self.last_out = (0, 0.0, 0.0, 0.0)

S = Shared()


def scan_cb(msg):
    with S.lock:
        S.scan = msg
        S.scan_stamp = time.time()


def sector_values(scan, center_deg, width_deg):
    vals = []
    center = math.radians(center_deg)
    half = math.radians(width_deg) * 0.5
    amin = scan.angle_min
    inc = scan.angle_increment
    for i, r in enumerate(scan.ranges):
        try:
            rf = float(r)
        except Exception:
            continue
        if not math.isfinite(rf):
            continue
        if rf < scan.range_min or rf > scan.range_max:
            continue
        a = norm_angle(amin + i * inc)
        if abs(norm_angle(a - center)) <= half:
            vals.append(rf)
    vals.sort()
    return vals


def sector_p(scan, center_deg, width_deg, percentile):
    vals = sector_values(scan, center_deg, width_deg)
    if not vals:
        return 999.0
    idx = int(clamp(percentile, 0.0, 1.0) * (len(vals) - 1))
    return vals[idx]


def read_raw_cmd():
    try:
        with open(RAW_CMD_PATH, "r") as f:
            parts = f.read().strip().split()
        if len(parts) < 5:
            return 0, 0.0, 0.0, 0.0, 0.0
        en = int(float(parts[0]))
        vx = float(parts[1]); vy = float(parts[2]); wz = float(parts[3]); st = float(parts[4])
        return en, vx, vy, wz, st
    except Exception:
        return 0, 0.0, 0.0, 0.0, 0.0


def write_cmd(enable, vx, vy, wz):
    vx = clamp(float(vx), -0.04, 0.24)
    vy = clamp(float(vy), -0.14, 0.14)
    wz = clamp(float(wz), -0.60, 0.60)
    with open(OUT_CMD_PATH, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (1 if enable else 0, vx, vy, wz, time.time()))


def scale_near(x, hard, soft):
    if x <= hard:
        return 0.0
    if x >= soft:
        return 1.0
    return clamp((x - hard) / max(1e-6, (soft - hard)), 0.0, 1.0)


def apply_body_filter(enable, vx, vy, wz, scan):
    # Physical footprint approximation. Tune for your A1 + payload.
    # The LiDAR sees the world, but the rear legs occupy space behind the LiDAR; therefore
    # rear-side sectors are intentionally conservative.
    front_stop = rospy.get_param("~front_stop_m", 0.70)
    front_slow = rospy.get_param("~front_slow_m", 1.05)
    side_hard = rospy.get_param("~side_hard_m", 0.42)
    side_soft = rospy.get_param("~side_soft_m", 0.58)
    rear_hard = rospy.get_param("~rear_side_hard_m", 0.48)
    rear_soft = rospy.get_param("~rear_side_soft_m", 0.66)
    desired_side = rospy.get_param("~desired_side_m", 0.72)
    max_vx_side = rospy.get_param("~max_vx_when_side_close", 0.045)
    k_side_vy = rospy.get_param("~k_side_vy", 0.16)
    max_side_vy = rospy.get_param("~max_side_vy", 0.075)
    natural_vy_bias = rospy.get_param("~vy_bias", -0.035)
    wz_bias = rospy.get_param("~wz_bias", 0.0)
    vy_sign = rospy.get_param("~vy_sign", 1.0)
    wz_sign = rospy.get_param("~wz_sign", 1.0)
    rear_swing_protect = rospy.get_param("~rear_swing_protect", True)

    front = sector_p(scan, 0, 35, 0.10)
    left = min(sector_p(scan, 70, 55, 0.15), sector_p(scan, 100, 45, 0.15))
    right = min(sector_p(scan, -70, 55, 0.15), sector_p(scan, -100, 45, 0.15))
    left_front = sector_p(scan, 35, 35, 0.15)
    right_front = sector_p(scan, -35, 35, 0.15)
    left_rear = sector_p(scan, 145, 55, 0.15)
    right_rear = sector_p(scan, -145, 55, 0.15)

    reason = []
    vx0, vy0, wz0 = vx, vy, wz

    # Constant compensation for observed natural left drift.
    # If this worsens drift, invert _vy_bias or _vy_sign.
    vy += vy_sign * natural_vy_bias
    wz += wz_sign * wz_bias

    # Front safety: match the previously confirmed behavior, but let the original driver
    # still perform its own stop as a second layer.
    if front < front_stop:
        if vx > 0.0:
            vx = 0.0
        reason.append("front_stop")
    elif front < front_slow and vx > 0.0:
        vx *= scale_near(front, front_stop, front_slow)
        reason.append("front_slow")

    # Side footprint safety. If a wall/furniture is near the left side, command rightward
    # side motion and suppress commands that move further left. Symmetric for right side.
    left_min = min(left, left_front, left_rear)
    right_min = min(right, right_front, right_rear)

    side_corr = 0.0
    if left_min < desired_side:
        side_corr += -k_side_vy * (desired_side - left_min)  # move right
        reason.append("left_clear")
    if right_min < desired_side:
        side_corr += +k_side_vy * (desired_side - right_min)  # move left
        reason.append("right_clear")
    side_corr = clamp(side_corr, -max_side_vy, max_side_vy)
    vy += vy_sign * side_corr

    # Do not allow command to move into a close side wall.
    if left_min < side_soft and vy > 0.0:
        vy = min(vy, 0.0)
        reason.append("block_left_vy")
    if right_min < side_soft and vy < 0.0:
        vy = max(vy, 0.0)
        reason.append("block_right_vy")

    # Slow forward when the body envelope is close to either side.
    min_side = min(left_min, right_min)
    if min_side < side_hard and vx > 0.0:
        vx = 0.0
        reason.append("side_hard_stop_vx")
    elif min_side < side_soft and vx > max_vx_side:
        vx = max_vx_side
        reason.append("side_slow_vx")

    # Rear-leg swing protection during yaw. With +wz, rear-left tends to move right;
    # with -wz, rear-left tends to move left. Therefore left rear close blocks right yaw.
    if rear_swing_protect:
        if left_rear < rear_hard and wz < 0.0:
            wz = 0.0
            reason.append("left_rear_block_wz")
        elif left_rear < rear_soft and wz < 0.0:
            wz *= scale_near(left_rear, rear_hard, rear_soft)
            reason.append("left_rear_slow_wz")
        if right_rear < rear_hard and wz > 0.0:
            wz = 0.0
            reason.append("right_rear_block_wz")
        elif right_rear < rear_soft and wz > 0.0:
            wz *= scale_near(right_rear, rear_hard, rear_soft)
            reason.append("right_rear_slow_wz")

    # If both sides are narrow, prefer straight slow motion rather than diagonal scraping.
    corridor_width_est = left_min + right_min
    if corridor_width_est < rospy.get_param("~narrow_corridor_m", 1.05):
        vy = clamp(vy, -0.030, 0.030)
        if vx > 0.04:
            vx = 0.04
        reason.append("narrow_corridor")

    # Conservative final clamps.
    vx = clamp(vx, -0.02, 0.20)
    vy = clamp(vy, -0.10, 0.10)
    wz = clamp(wz, -0.50, 0.50)

    if not reason:
        reason = ["pass"]
    dbg = ("reason=%s raw=%.3f %.3f %.3f out=%.3f %.3f %.3f "
           "front=%.3f left=%.3f right=%.3f lf=%.3f rf=%.3f lr=%.3f rr=%.3f") % (
        "+".join(reason), vx0, vy0, wz0, vx, vy, wz,
        front, left_min, right_min, left_front, right_front, left_rear, right_rear)
    return enable, vx, vy, wz, dbg


def main():
    rospy.init_node("a1_body_safety_filter")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.loginfo("a1_body_safety_filter started scan=%s raw=%s out=%s", scan_topic, RAW_CMD_PATH, OUT_CMD_PATH)
    rate = rospy.Rate(rospy.get_param("~rate", 25.0))
    last_dbg = 0.0
    while not rospy.is_shutdown():
        en, vx, vy, wz, st = read_raw_cmd()
        now = time.time()
        with S.lock:
            scan = S.scan
            scan_age = now - S.scan_stamp if S.scan_stamp else 999.0
        if en == 0 or st <= 0.0 or now - st > rospy.get_param("~raw_timeout", 0.80):
            write_cmd(0, 0.0, 0.0, 0.0)
            dbg = "reason=disabled_or_timeout raw_age=%.3f" % (now - st if st else 999.0)
        elif scan is None or scan_age > rospy.get_param("~scan_timeout", 0.80):
            # No scan means no body-size safety. Fail closed.
            write_cmd(0, 0.0, 0.0, 0.0)
            dbg = "reason=no_recent_scan scan_age=%.3f" % scan_age
        else:
            en2, vx2, vy2, wz2, dbg = apply_body_filter(en, vx, vy, wz, scan)
            write_cmd(en2, vx2, vy2, wz2)
        try:
            with open(DEBUG_PATH, "w") as f:
                f.write(dbg + "\n")
        except Exception:
            pass
        if now - last_dbg > 0.5:
            last_dbg = now
            rospy.loginfo(dbg)
        rate.sleep()


if __name__ == "__main__":
    main()
