#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A1 footprint-aware body safety filter for ROS Melodic / Python2.

Input : /tmp/a1_follow_cmd_raw   (enable vx vy wz stamp)
Output: /tmp/a1_follow_cmd       (filtered command for a1_high_follow_driver)
Debug : /tmp/a1_body_safety_debug

This version treats the A1 as a finite swept footprint instead of a point.
It uses 2D LiDAR points in the laser frame and checks front/side/rear-corner
clearance around an inflated walking envelope.

Coordinate convention:
  x: forward from LiDAR
  y: left from LiDAR
  +vx forward, +vy robot-left, +wz left-yaw.
"""
from __future__ import print_function

import math
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
    for i, r in enumerate(msg.ranges):
        try:
            rf = float(r)
        except Exception:
            a += inc
            continue
        if math.isfinite(rf) and msg.range_min <= rf <= msg.range_max:
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
    vx = clamp(float(vx), -0.04, 0.24)
    vy = clamp(float(vy), -0.14, 0.14)
    wz = clamp(float(wz), -0.60, 0.60)
    with open(OUT_CMD_PATH, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (1 if enable else 0, vx, vy, wz, time.time()))


def rect_front_metrics(points, rear_x, front_x, half_w, front_stop, front_slow):
    # Points inside the robot's swept forward corridor.
    # Returns closest forward x distance to obstacle and side-specific minima.
    fxs = []
    for x, y, r, a in points:
        if 0.02 <= x <= front_slow and abs(y) <= half_w:
            fxs.append(x)
    return percentile(fxs, 0.10, 999.0)


def side_clearances(points, rear_x, front_x, half_w):
    # Clearance means distance from inflated footprint side boundary, not from LiDAR center.
    # Positive clearance is free space outside footprint.
    left_c = []
    right_c = []
    left_rear_c = []
    right_rear_c = []
    left_front_c = []
    right_front_c = []
    for x, y, r, a in points:
        if rear_x <= x <= front_x:
            if y > half_w:
                left_c.append(y - half_w)
                if x < -0.10:
                    left_rear_c.append(y - half_w)
                if x > -0.05:
                    left_front_c.append(y - half_w)
            elif y < -half_w:
                right_c.append((-half_w) - y)
                if x < -0.10:
                    right_rear_c.append((-half_w) - y)
                if x > -0.05:
                    right_front_c.append((-half_w) - y)
    q = 0.10
    return {
        "left": percentile(left_c, q),
        "right": percentile(right_c, q),
        "left_rear": percentile(left_rear_c, q),
        "right_rear": percentile(right_rear_c, q),
        "left_front": percentile(left_front_c, q),
        "right_front": percentile(right_front_c, q),
    }


def sector_front_min(points, width_deg=35.0):
    half = math.radians(width_deg) * 0.5
    vals = [r for x, y, r, a in points if abs(norm_angle(a)) <= half]
    return percentile(vals, 0.10, 999.0)


def scale_near(x, hard, soft):
    if x <= hard:
        return 0.0
    if x >= soft:
        return 1.0
    return clamp((x - hard) / max(1e-6, soft - hard), 0.0, 1.0)


def filter_cmd(enable, vx, vy, wz, points):
    # A1 static size is roughly 0.50 x 0.30 x 0.40 m. Walking envelope is larger.
    # Defaults assume the 2D LiDAR is near the front upper body, about 0.20 m ahead
    # of the body center. Tune lidar_x_from_body_center after measuring.
    body_len = rospy.get_param("~body_length_m", 0.50)
    body_w = rospy.get_param("~body_width_m", 0.30)
    lidar_x = rospy.get_param("~lidar_x_from_body_center_m", 0.20)  # + means LiDAR is forward of body center
    dyn_front = rospy.get_param("~dynamic_front_extra_m", 0.17)
    dyn_rear = rospy.get_param("~dynamic_rear_extra_m", 0.25)
    dyn_side = rospy.get_param("~dynamic_side_extra_m", 0.14)
    safety_margin = rospy.get_param("~safety_margin_m", 0.08)

    front_x = body_len * 0.5 - lidar_x + dyn_front
    rear_x = -body_len * 0.5 - lidar_x - dyn_rear
    half_w = body_w * 0.5 + dyn_side + safety_margin

    front_stop = rospy.get_param("~front_stop_m", 0.70)
    front_slow = rospy.get_param("~front_slow_m", 1.05)

    side_hard = rospy.get_param("~side_hard_clearance_m", 0.08)
    side_soft = rospy.get_param("~side_soft_clearance_m", 0.22)
    desired_side = rospy.get_param("~desired_side_clearance_m", 0.30)
    rear_hard = rospy.get_param("~rear_hard_clearance_m", 0.10)
    rear_soft = rospy.get_param("~rear_soft_clearance_m", 0.26)

    k_side_vy = rospy.get_param("~k_side_vy", 0.075)
    max_side_vy = rospy.get_param("~max_side_vy", 0.050)
    max_vx_side = rospy.get_param("~max_vx_when_side_close", 0.090)
    max_vx_narrow = rospy.get_param("~max_vx_when_narrow", 0.055)

    natural_vy_bias = rospy.get_param("~vy_bias", -0.035)
    vy_sign = rospy.get_param("~vy_sign", 1.0)
    wz_bias = rospy.get_param("~wz_bias", 0.0)
    wz_sign = rospy.get_param("~wz_sign", 1.0)
    rear_swing_protect = rospy.get_param("~rear_swing_protect", True)

    vx0, vy0, wz0 = vx, vy, wz
    reason = []

    # Add calibrated drift compensation first. This is not obstacle avoidance;
    # it counteracts the observed constant left drift.
    vy += vy_sign * natural_vy_bias
    wz += wz_sign * wz_bias

    front_sector = sector_front_min(points, 35.0)
    front_rect = rect_front_metrics(points, rear_x, front_x, half_w, front_stop, front_slow)
    # Use the more conservative of the sector and body-width rectangle in front.
    front = min(front_sector, front_rect)

    cs = side_clearances(points, rear_x, front_x, half_w)
    left = min(cs["left"], cs["left_front"], cs["left_rear"])
    right = min(cs["right"], cs["right_front"], cs["right_rear"])

    # Front safety: keep the original proven 0.70 m stop behavior.
    if front < front_stop:
        if vx > 0.0:
            vx = 0.0
        reason.append("front_stop")
    elif front < front_slow and vx > 0.0:
        vx *= scale_near(front, front_stop, front_slow)
        reason.append("front_slow")

    # Side clearance: do not stop immediately. First bias away and reduce vx.
    side_corr = 0.0
    if left < desired_side:
        side_corr += -k_side_vy * (desired_side - left)
        reason.append("left_clear")
    if right < desired_side:
        side_corr += +k_side_vy * (desired_side - right)
        reason.append("right_clear")
    side_corr = clamp(side_corr, -max_side_vy, max_side_vy)
    vy += vy_sign * side_corr

    # Block lateral commands that go into a close side.
    if left < side_soft and vy > 0.0:
        vy = min(vy, 0.0)
        reason.append("block_left_vy")
    if right < side_soft and vy < 0.0:
        vy = max(vy, 0.0)
        reason.append("block_right_vy")

    min_side = min(left, right)
    if min_side < side_hard and vx > max_vx_narrow:
        vx = max_vx_narrow
        reason.append("side_hard_slow_vx")
    elif min_side < side_soft and vx > max_vx_side:
        vx = max_vx_side
        reason.append("side_soft_slow_vx")

    # Rear swing protection using rear corner lateral velocity approximation.
    # For a rear corner at x=rear_x, lateral velocity is vy + wz * rear_x.
    if rear_swing_protect:
        left_rear = cs["left_rear"]
        right_rear = cs["right_rear"]
        rear_left_corner_vy = vy + wz * rear_x
        rear_right_corner_vy = vy + wz * rear_x
        if left_rear < rear_hard and rear_left_corner_vy > 0.0:
            if vy > 0.0:
                vy = 0.0
            if wz < 0.0:
                wz = 0.0
            reason.append("left_rear_block")
        elif left_rear < rear_soft and rear_left_corner_vy > 0.0:
            if wz < 0.0:
                wz *= scale_near(left_rear, rear_hard, rear_soft)
            reason.append("left_rear_slow")
        if right_rear < rear_hard and rear_right_corner_vy < 0.0:
            if vy < 0.0:
                vy = 0.0
            if wz > 0.0:
                wz = 0.0
            reason.append("right_rear_block")
        elif right_rear < rear_soft and rear_right_corner_vy < 0.0:
            if wz > 0.0:
                wz *= scale_near(right_rear, rear_hard, rear_soft)
            reason.append("right_rear_slow")

    # Final clamps. Do not zero visual-follow unnecessarily.
    vx = clamp(vx, -0.02, 0.20)
    vy = clamp(vy, -0.10, 0.10)
    wz = clamp(wz, -0.50, 0.50)

    if not reason:
        reason = ["pass"]
    dbg = ("reason=%s raw=%.3f %.3f %.3f out=%.3f %.3f %.3f "
           "front=%.3f front_sector=%.3f front_rect=%.3f left_clear=%.3f right_clear=%.3f "
           "lr_clear=%.3f rr_clear=%.3f footprint=[x %.2f..%.2f y +/-%.2f]") % (
        "+".join(reason), vx0, vy0, wz0, vx, vy, wz,
        front, front_sector, front_rect, left, right, cs["left_rear"], cs["right_rear"], rear_x, front_x, half_w)
    return enable, vx, vy, wz, dbg


def main():
    rospy.init_node("a1_body_safety_filter_footprint_v2")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.loginfo("a1_body_safety_filter_footprint_v2 started scan=%s raw=%s out=%s", scan_topic, RAW_CMD_PATH, OUT_CMD_PATH)
    rate = rospy.Rate(rospy.get_param("~rate", 25.0))
    last_log = 0.0

    # Raw command can momentarily become enable=0 when the vision client drops a frame
    # or when the HTTP writer updates the file. Do not immediately zero the final
    # command; keep the last valid command briefly. A real stop still takes effect
    # after stop_hold_s.
    last_valid = None  # (vx, vy, wz, time_received)

    while not rospy.is_shutdown():
        en, vx, vy, wz, st = read_raw_cmd()
        now = time.time()
        raw_age = now - st if st else 999.0
        raw_timeout = rospy.get_param("~raw_timeout", 1.20)
        stop_hold_s = rospy.get_param("~stop_hold_s", 0.35)
        scan_timeout = rospy.get_param("~scan_timeout", 1.20)

        raw_is_fresh = (st > 0.0 and raw_age <= raw_timeout)
        use_hold = False

        if en != 0 and raw_is_fresh:
            last_valid = (vx, vy, wz, now)
            use_en, use_vx, use_vy, use_wz = 1, vx, vy, wz
        elif last_valid is not None and (now - last_valid[3]) <= stop_hold_s:
            # Short zero burst; keep previous valid command.
            use_en = 1
            use_vx, use_vy, use_wz = last_valid[0], last_valid[1], last_valid[2]
            use_hold = True
        else:
            write_cmd(0, 0.0, 0.0, 0.0)
            dbg = "reason=disabled_or_timeout raw_en=%d raw_age=%.3f last_valid_age=%.3f" % (
                en, raw_age, (now - last_valid[3]) if last_valid is not None else 999.0)
            try:
                with open(DEBUG_PATH, "w") as f:
                    f.write(dbg + "\n")
            except Exception:
                pass
            if now - last_log > 0.5:
                last_log = now
                rospy.loginfo(dbg)
            rate.sleep()
            continue

        with S.lock:
            points = list(S.points)
            scan_age = now - S.stamp if S.stamp else 999.0

        if not points or scan_age > scan_timeout:
            # During temporary scan dropouts, do not kill follow; pass through held/raw
            # command. The Unitree driver and obstacle writer still provide front stop
            # when /tmp/a1_obstacle_front_m exists.
            write_cmd(use_en, use_vx, use_vy, use_wz)
            dbg = "reason=scan_missing_passthrough scan_age=%.3f raw_age=%.3f hold=%d out=%.3f %.3f %.3f" % (
                scan_age, raw_age, 1 if use_hold else 0, use_vx, use_vy, use_wz)
        else:
            en2, vx2, vy2, wz2, dbg = filter_cmd(use_en, use_vx, use_vy, use_wz, points)
            if use_hold:
                dbg = "hold_last+" + dbg
            write_cmd(en2, vx2, vy2, wz2)
        try:
            with open(DEBUG_PATH, "w") as f:
                f.write(dbg + "\n")
        except Exception:
            pass
        if now - last_log > 0.5:
            last_log = now
            rospy.loginfo(dbg)
        rate.sleep()


if __name__ == "__main__":
    main()
