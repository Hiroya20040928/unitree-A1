#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
A1 SLAM/LiDAR route-fallback node for ROS Melodic (Python 2 compatible).

Purpose:
- PC MediaPipe client drives A1 directly while person is visible.
- When the PC reports mode=lost, this node takes over and writes /tmp/a1_follow_cmd.
- It uses /scan and /odom to continue a short distance along the locally open corridor,
  while keeping away from walls and stopping after a bounded time/distance.

Run on NX:
  source /opt/ros/melodic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_IP=192.168.12.1
  python ~/a1_slam_route_fallback_node.py _scan_topic:=/scan _odom_topic:=/odom

PC client sends state to:
  http://192.168.12.1:8091/route_state?mode=visual&xerr=0.12&area=0.23
  http://192.168.12.1:8091/route_state?mode=lost&xerr=0.12
  http://192.168.12.1:8091/route_state?mode=stop
"""
from __future__ import print_function

import math
import os
import threading
import time

try:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
    import urlparse
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
    import urllib.parse as urlparse

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

FOLLOW_CMD_PATH = "/tmp/a1_follow_cmd"
OBSTACLE_PATH = "/tmp/a1_obstacle_front_m"
DEBUG_PATH = "/tmp/a1_route_debug"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_from_quat(q):
    # geometry_msgs/Quaternion -> yaw
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SharedState(object):
    def __init__(self):
        self.lock = threading.RLock()
        self.mode = "stop"       # stop | visual | lost
        self.xerr = 0.0
        self.area = 0.0
        self.http_stamp = 0.0
        self.last_visual_stamp = 0.0
        self.last_visual_xerr = 0.0
        self.lost_start_time = 0.0
        self.lost_start_pose = None
        self.last_scan = None
        self.last_scan_stamp = 0.0
        self.pose = None          # (x,y,yaw)
        self.last_odom_stamp = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.route_active = False

    def set_http_state(self, mode, xerr, area):
        now = time.time()
        with self.lock:
            prev = self.mode
            self.mode = mode
            self.xerr = xerr
            self.area = area
            self.http_stamp = now
            if mode == "visual":
                self.last_visual_stamp = now
                self.last_visual_xerr = xerr
                self.route_active = False
            elif mode == "lost":
                # Start a new bounded route-search segment only at transition.
                if prev != "lost" or self.lost_start_time <= 0.0:
                    self.lost_start_time = now
                    self.lost_start_pose = self.pose
                self.route_active = True
                if abs(xerr) > 1e-4:
                    self.last_visual_xerr = xerr
            elif mode == "stop":
                self.route_active = False
                self.lost_start_time = 0.0
                self.lost_start_pose = None
                self.last_cmd = (0.0, 0.0, 0.0)


S = SharedState()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_text(self, code, body):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        if isinstance(body, unicode):  # noqa: F821  (Python2 only)
            body = body.encode("utf-8")
        elif not isinstance(body, bytes):
            body = str(body).encode("utf-8")
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse.urlparse(self.path)
        qs = urlparse.parse_qs(parsed.query)
        path = parsed.path
        if path in ("/", "/status"):
            with S.lock:
                body = "OK\nmode=%s\nxerr=%.4f\narea=%.4f\nroute_active=%d\n" % (
                    S.mode, S.xerr, S.area, 1 if S.route_active else 0)
            self.send_text(200, body)
            return
        if path == "/route_state":
            mode = qs.get("mode", ["stop"])[0]
            if mode not in ("stop", "visual", "lost"):
                self.send_text(400, "bad mode\n")
                return
            try:
                xerr = float(qs.get("xerr", ["0"])[0])
            except Exception:
                xerr = 0.0
            try:
                area = float(qs.get("area", ["0"])[0])
            except Exception:
                area = 0.0
            S.set_http_state(mode, xerr, area)
            if mode == "stop":
                write_follow_cmd(0, 0.0, 0.0, 0.0)
            self.send_text(200, "mode=%s xerr=%.4f area=%.4f\n" % (mode, xerr, area))
            return
        self.send_text(404, "not found\n")


def start_http_server(port):
    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    th = threading.Thread(target=srv.serve_forever)
    th.daemon = True
    th.start()
    rospy.loginfo("route_state HTTP server started on port %d", port)


def write_follow_cmd(enable, vx, vy, wz):
    vx = clamp(float(vx), -0.02, 0.22)
    vy = clamp(float(vy), -0.10, 0.10)
    wz = clamp(float(wz), -0.50, 0.50)
    with open(FOLLOW_CMD_PATH, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (1 if enable else 0, vx, vy, wz, time.time()))


def read_front_obstacle():
    try:
        with open(OBSTACLE_PATH, "r") as f:
            s = f.read().strip()
        if s == "inf" or s == "":
            return 999.0
        return float(s)
    except Exception:
        return 999.0


def scan_cb(msg):
    with S.lock:
        S.last_scan = msg
        S.last_scan_stamp = time.time()


def odom_cb(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    pose = (float(p.x), float(p.y), yaw_from_quat(q))
    with S.lock:
        S.pose = pose
        S.last_odom_stamp = time.time()


def sector_values(scan, center_rad, half_width_rad):
    vals = []
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
        if abs(norm_angle(a - center_rad)) <= half_width_rad:
            vals.append(rf)
    return vals


def sector_min(scan, center_deg, width_deg):
    vals = sector_values(scan, math.radians(center_deg), math.radians(width_deg) * 0.5)
    if not vals:
        return 999.0
    vals.sort()
    return vals[max(0, int(0.10 * (len(vals) - 1)))]


def sector_median(scan, center_deg, width_deg):
    vals = sector_values(scan, math.radians(center_deg), math.radians(width_deg) * 0.5)
    if not vals:
        return 999.0
    vals.sort()
    return vals[len(vals) // 2]


def distance_from_start(pose, start_pose):
    if pose is None or start_pose is None:
        return 0.0
    return math.hypot(pose[0] - start_pose[0], pose[1] - start_pose[1])


def compute_route_cmd(scan, pose, lost_age, lost_dist, last_xerr):
    # Parameters can be tuned at launch with _param:=value.
    max_vx = rospy.get_param("~route_max_vx", 0.045)
    align_vx = rospy.get_param("~route_align_vx", 0.020)
    max_wz = rospy.get_param("~route_max_wz", 0.16)
    k_heading = rospy.get_param("~k_heading", 0.40)
    heading_sign = rospy.get_param("~heading_sign", 1.0)
    side_pref_gain = rospy.get_param("~side_pref_gain", 0.26)
    heading_penalty = rospy.get_param("~heading_penalty", 0.28)
    hard_front = rospy.get_param("~hard_front_m", 0.52)
    slow_front = rospy.get_param("~slow_front_m", 0.95)
    min_clear = rospy.get_param("~min_candidate_clear_m", 0.62)
    desired_side = rospy.get_param("~desired_side_m", 0.70)
    k_side_vy = rospy.get_param("~k_side_vy", 0.025)
    k_wall_yaw = rospy.get_param("~k_wall_yaw", 0.02)
    vy_bias = rospy.get_param("~vy_bias", 0.000)
    scan_side_sign = rospy.get_param("~scan_side_sign", -1.0)

    front = sector_min(scan, 0, 30)
    left = sector_median(scan, 75, 30)
    right = sector_median(scan, -75, 30)
    left_front = sector_median(scan, 35, 25)
    right_front = sector_median(scan, -35, 25)

    # Candidate free-space heading in robot frame. Positive is left in ROS convention.
    pref_angle = 0.0
    if abs(last_xerr) > 0.04:
        # Image xerr positive = person on image-right. With usual A1 setup this means right turn,
        # i.e. negative scan angle. scan_side_sign=-1 implements that default.
        pref_angle = scan_side_sign * math.copysign(math.radians(45.0), last_xerr)

    best_theta = 0.0
    best_range = -1.0
    best_score = -999.0
    for deg in range(-75, 80, 5):
        theta = math.radians(deg)
        r = sector_median(scan, deg, 18)
        if r < min_clear:
            continue
        score = r
        score -= heading_penalty * abs(theta)
        score -= side_pref_gain * abs(norm_angle(theta - pref_angle))
        # Prefer not scraping walls diagonally.
        if deg > 20 and left_front < desired_side:
            score -= (desired_side - left_front) * 0.60
        if deg < -20 and right_front < desired_side:
            score -= (desired_side - right_front) * 0.60
        if score > best_score:
            best_score = score
            best_theta = theta
            best_range = r

    if best_range < min_clear:
        return 0.0, 0.0, 0.0, "route_no_clear", front, left, right, best_theta, best_range

    # Forward speed: move slowly into the blind area, but do not push into a close front obstacle.
    if front < hard_front:
        vx = 0.0
    elif front < slow_front or abs(best_theta) > math.radians(28):
        vx = align_vx
    else:
        vx = max_vx

    # Heading command toward open corridor.
    wz = heading_sign * k_heading * best_theta

    # Wall centering. +vy is robot-left. If left wall is closer, move right: negative vy.
    side_err = right - left
    if left > 5.0 and right > 5.0:
        vy_wall = 0.0
    elif left > 5.0:
        vy_wall = +0.020
    elif right > 5.0:
        vy_wall = -0.020
    else:
        vy_wall = -k_side_vy * side_err

    # Yaw away from closer front-side wall.
    if left_front < 5.0 and right_front < 5.0:
        wz += -k_wall_yaw * (right_front - left_front)

    vy = clamp(vy_bias + vy_wall, -0.09, 0.09)
    wz = clamp(wz, -max_wz, max_wz)
    return vx, vy, wz, "route_follow", front, left, right, best_theta, best_range


def smooth(prev, target, alpha):
    return prev + alpha * (target - prev)


def main_loop():
    http_port = int(rospy.get_param("~http_port", 8091))
    max_lost_time = rospy.get_param("~max_lost_time", 2.8)
    max_lost_dist = rospy.get_param("~max_lost_dist", 0.75)
    state_timeout = rospy.get_param("~state_timeout", 1.0)
    alpha = rospy.get_param("~cmd_alpha", 0.25)
    rate_hz = rospy.get_param("~rate", 20.0)

    start_http_server(http_port)
    rate = rospy.Rate(rate_hz)
    last_log = 0.0
    stop_written = False

    while not rospy.is_shutdown():
        now = time.time()
        with S.lock:
            mode = S.mode
            xerr = S.xerr
            last_xerr = S.last_visual_xerr
            scan = S.last_scan
            scan_age = now - S.last_scan_stamp if S.last_scan_stamp else 999.0
            pose = S.pose
            lost_start_time = S.lost_start_time
            lost_start_pose = S.lost_start_pose
            state_age = now - S.http_stamp if S.http_stamp else 999.0
            prev_cmd = S.last_cmd

        front_file = read_front_obstacle()
        reason = "idle"
        cmd = (0.0, 0.0, 0.0)

        if mode == "lost" and state_age < state_timeout and scan is not None and scan_age < 0.8:
            lost_age = now - lost_start_time if lost_start_time > 0.0 else 999.0
            lost_dist = distance_from_start(pose, lost_start_pose)
            if lost_age <= max_lost_time and lost_dist <= max_lost_dist:
                vx, vy, wz, reason, front, left, right, best_theta, best_range = compute_route_cmd(
                    scan, pose, lost_age, lost_dist, last_xerr)
                # Hard stop from existing obstacle writer. Still allow rotation if not dangerously close.
                if front_file < rospy.get_param("~absolute_stop_m", 0.48):
                    vx, vy, wz = 0.0, 0.0, 0.0
                    reason = "absolute_stop"
                # Smooth fallback commands.
                vx = smooth(prev_cmd[0], vx, alpha)
                vy = smooth(prev_cmd[1], vy, alpha)
                wz = smooth(prev_cmd[2], wz, alpha)
                cmd = (vx, vy, wz)
                write_follow_cmd(1, vx, vy, wz)
                stop_written = False
            else:
                reason = "lost_limit_stop t=%.2f d=%.2f" % (lost_age, lost_dist)
                write_follow_cmd(0, 0.0, 0.0, 0.0)
                stop_written = True
        elif mode == "stop" or state_age >= state_timeout:
            reason = "stop_or_state_timeout"
            if not stop_written:
                write_follow_cmd(0, 0.0, 0.0, 0.0)
                stop_written = True
        else:
            # visual mode: PC client owns /tmp/a1_follow_cmd.
            reason = "visual_passthrough"
            stop_written = False

        with S.lock:
            S.last_cmd = cmd

        if now - last_log > 0.5:
            last_log = now
            dbg = "mode=%s reason=%s cmd=%.3f %.3f %.3f xerr=%.3f front_file=%.3f" % (
                mode, reason, cmd[0], cmd[1], cmd[2], last_xerr, front_file)
            rospy.loginfo(dbg)
            try:
                with open(DEBUG_PATH, "w") as f:
                    f.write(dbg + "\n")
            except Exception:
                pass
        rate.sleep()


def main():
    rospy.init_node("a1_slam_route_fallback_node")
    scan_topic = rospy.get_param("~scan_topic", "/scan")
    odom_topic = rospy.get_param("~odom_topic", "/odom")
    rospy.Subscriber(scan_topic, LaserScan, scan_cb, queue_size=1)
    rospy.Subscriber(odom_topic, Odometry, odom_cb, queue_size=1)
    rospy.loginfo("a1_slam_route_fallback_node started scan=%s odom=%s", scan_topic, odom_topic)
    main_loop()


if __name__ == "__main__":
    main()
