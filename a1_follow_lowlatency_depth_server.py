#!/usr/bin/env python3
"""
Low-latency A1 camera/follow server.

Features:
- /snapshot.jpg returns the latest RGB frame only, no MJPEG buffering.
- /follow writes /tmp/a1_follow_cmd for the existing high-level driver.
- /follow_stop writes a zero command.
- Optional RealSense depth support if pyrealsense2 is installed on NX.
- /depth_stats returns median/min depth in an ROI in the same 640x360 image coordinates.

Run on NX:
  python3 ~/a1_follow_lowlatency_depth_server.py
"""
import json
import math
import os
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import numpy as np

CAM_ID = 2
PORT = 8090
OUT_W = 640
OUT_H = 360
FOLLOW_CMD_PATH = "/tmp/a1_follow_cmd"

latest_jpeg = None
latest_color = None
latest_depth_m = None   # float32, aligned to latest_color, meters, shape OUT_H x OUT_W
latest_stamp = 0.0
latest_lock = threading.Lock()

rs_available = False
rs_error = "not initialized"

try:
    import pyrealsense2 as rs  # type: ignore
    rs_available = True
    rs_error = ""
except Exception as e:
    rs = None
    rs_available = False
    rs_error = repr(e)


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def encode_and_store(color_bgr, depth_m=None):
    global latest_jpeg, latest_color, latest_depth_m, latest_stamp

    color_bgr = cv2.resize(color_bgr, (OUT_W, OUT_H))
    if depth_m is not None:
        depth_m = cv2.resize(depth_m, (OUT_W, OUT_H), interpolation=cv2.INTER_NEAREST)

    ok, jpg = cv2.imencode(".jpg", color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if not ok:
        return

    with latest_lock:
        latest_jpeg = jpg.tobytes()
        latest_color = color_bgr
        latest_depth_m = depth_m
        latest_stamp = time.time()


def camera_loop_realsense():
    global rs_error
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("[INFO] RealSense opened. depth_scale=", depth_scale)

        while True:
            frames = pipeline.wait_for_frames(1000)
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            depth_m = depth_raw * float(depth_scale)
            encode_and_store(color, depth_m)

    except Exception as e:
        rs_error = repr(e)
        print("[WARN] RealSense path failed:", rs_error)
        print("[WARN] Falling back to OpenCV /dev/video%d color only" % CAM_ID)
        camera_loop_opencv()


def camera_loop_opencv():
    cap = cv2.VideoCapture(CAM_ID)
    if not cap.isOpened():
        print("[ERR] OpenCV camera could not be opened:", CAM_ID)
        return
    print("[INFO] OpenCV camera opened:", CAM_ID, "depth=unavailable")

    # Keep buffer small if backend supports it.
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] no frame")
            time.sleep(0.05)
            continue
        encode_and_store(frame, None)
        time.sleep(0.005)


def camera_loop():
    if rs_available:
        camera_loop_realsense()
    else:
        print("[WARN] pyrealsense2 unavailable:", rs_error)
        camera_loop_opencv()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def write_follow_cmd(enable, vx, vy, wz):
    # Hard clamp at server side. Driver should also clamp.
    enable_i = 1 if enable else 0
    vx = clamp(float(vx), -0.30, 0.30)
    vy = clamp(float(vy), -0.20, 0.20)
    wz = clamp(float(wz), -0.60, 0.60)
    stamp = time.time()
    with open(FOLLOW_CMD_PATH, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (enable_i, vx, vy, wz, stamp))
    return enable_i, vx, vy, wz, stamp


def get_depth_stats(x1, y1, x2, y2):
    with latest_lock:
        depth = None if latest_depth_m is None else latest_depth_m.copy()
        stamp = latest_stamp

    if depth is None:
        return {"ok": 0, "reason": "depth_unavailable", "rs_available": int(rs_available), "rs_error": rs_error}

    H, W = depth.shape[:2]
    x1 = int(clamp(x1, 0, W - 1))
    x2 = int(clamp(x2, 0, W))
    y1 = int(clamp(y1, 0, H - 1))
    y2 = int(clamp(y2, 0, H))
    if x2 <= x1 or y2 <= y1:
        return {"ok": 0, "reason": "bad_roi"}

    roi = depth[y1:y2, x1:x2]
    valid = roi[(roi > 0.15) & (roi < 8.0) & np.isfinite(roi)]
    if valid.size < 20:
        return {"ok": 0, "reason": "no_valid_depth", "n": int(valid.size)}

    return {
        "ok": 1,
        "stamp": stamp,
        "n": int(valid.size),
        "median_m": float(np.median(valid)),
        "min_m": float(np.percentile(valid, 5)),
        "p20_m": float(np.percentile(valid, 20)),
        "p80_m": float(np.percentile(valid, 80)),
    }


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_text(self, code, body):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(body.encode())

    def send_json(self, code, obj):
        body = json.dumps(obj, separators=(",", ":"))
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(body.encode())

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        q = urllib.parse.parse_qs(parsed.query)

        if path in ("/", "/status"):
            with latest_lock:
                age = time.time() - latest_stamp if latest_stamp else 999.0
                has_frame = latest_jpeg is not None
                has_depth = latest_depth_m is not None
            body = (
                "OK\n"
                "server=a1_follow_lowlatency_depth_server\n"
                "snapshot=/snapshot.jpg\n"
                "follow=/follow?enable=1&vx=0.10&vy=0&wz=0\n"
                "stop=/follow_stop\n"
                "depth=/depth_stats?x1=260&y1=120&x2=380&y2=300\n"
                "has_frame=%d\n" % int(has_frame) +
                "has_depth=%d\n" % int(has_depth) +
                "frame_age=%.3f\n" % age +
                "rs_available=%d\n" % int(rs_available) +
                "rs_error=%s\n" % rs_error
            )
            self.send_text(200, body)
            return

        if path == "/snapshot.jpg":
            with latest_lock:
                jpg = latest_jpeg
            if jpg is None:
                self.send_text(503, "no frame\n")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
            self.end_headers()
            self.wfile.write(jpg)
            return

        if path == "/follow":
            try:
                enable = int(q.get("enable", ["0"])[0]) != 0
                vx = float(q.get("vx", ["0"])[0])
                vy = float(q.get("vy", ["0"])[0])
                wz = float(q.get("wz", ["0"])[0])
                enable_i, vx, vy, wz, stamp = write_follow_cmd(enable, vx, vy, wz)
                self.send_text(200, "follow enable=%d vx=%.3f vy=%.3f wz=%.3f stamp=%.6f\n" % (enable_i, vx, vy, wz, stamp))
            except Exception as e:
                self.send_text(400, "bad follow command: %r\n" % e)
            return

        if path == "/follow_stop":
            enable_i, vx, vy, wz, stamp = write_follow_cmd(False, 0.0, 0.0, 0.0)
            self.send_text(200, "stopped stamp=%.6f\n" % stamp)
            return

        if path == "/depth_stats":
            try:
                x1 = float(q.get("x1", ["0"])[0])
                y1 = float(q.get("y1", ["0"])[0])
                x2 = float(q.get("x2", [str(OUT_W)])[0])
                y2 = float(q.get("y2", [str(OUT_H)])[0])
                self.send_json(200, get_depth_stats(x1, y1, x2, y2))
            except Exception as e:
                self.send_json(400, {"ok": 0, "reason": repr(e)})
            return

        self.send_text(404, "not found\n")


def main():
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("[INFO] low-latency follow server started")
    print("[INFO] status:   http://192.168.12.1:%d/status" % PORT)
    print("[INFO] snapshot: http://192.168.12.1:%d/snapshot.jpg" % PORT)
    print("[INFO] follow:   http://192.168.12.1:%d/follow?enable=1&vx=0.10&vy=0&wz=0" % PORT)
    print("[INFO] stop:     http://192.168.12.1:%d/follow_stop" % PORT)
    print("[INFO] depth:    http://192.168.12.1:%d/depth_stats?x1=260&y1=120&x2=380&y2=300" % PORT)
    server.serve_forever()


if __name__ == "__main__":
    main()
