#!/usr/bin/env python3
import cv2
import time
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

CAM_ID = 2
PORT = 8090

ACTION_PATH = "/tmp/a1_action"
TRIGGER_PATH = "/tmp/a1_choki_trigger"
FOLLOW_PATH = "/tmp/a1_follow_cmd"

latest_jpeg = None
latest_lock = threading.Lock()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def write_follow(enable, vx=0.0, vy=0.0, wz=0.0):
    # Conservative clamp on server side too.
    en = 1 if enable else 0
    vx = max(0.0, min(0.22, float(vx)))
    vy = max(-0.08, min(0.08, float(vy)))
    wz = max(-0.35, min(0.35, float(wz)))
    ts = time.monotonic()
    with open(FOLLOW_PATH, "w") as f:
        f.write("%d %.5f %.5f %.5f %.6f\n" % (en, vx, vy, wz, ts))


def write_action(name):
    allowed = {
        "ready", "prone", "wave", "shake", "sway", "stop",
        "emergency_prone", "choki", "follow_start", "follow_stop"
    }
    if name not in allowed:
        return False, "unknown action: " + name

    with open(ACTION_PATH, "w") as f:
        f.write(name + "\n")

    if name in ("choki", "prone", "emergency_prone"):
        with open(TRIGGER_PATH, "w") as f:
            f.write(name + "\n")

    if name in ("stop", "follow_stop", "prone", "emergency_prone"):
        write_follow(False)

    return True, "action written: " + name


def camera_loop():
    global latest_jpeg

    cap = cv2.VideoCapture(CAM_ID)
    if not cap.isOpened():
        print("[ERR] camera could not be opened:", CAM_ID)
        return

    print("[INFO] camera opened:", CAM_ID)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] no frame")
            time.sleep(0.1)
            continue

        frame = cv2.resize(frame, (640, 360))
        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            with latest_lock:
                latest_jpeg = jpg.tobytes()
        time.sleep(0.03)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_text(self, code, body):
        self.send_response(code)
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write((body + "\n").encode())

    def do_GET(self):
        global latest_jpeg
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/" or path == "/status":
            self.send_text(200,
                "OK\n"
                "/video\n"
                "/snapshot.jpg\n"
                "/action?name=ready\n"
                "/action?name=stop\n"
                "/follow?enable=1&vx=0.10&vy=0.00&wz=0.00\n"
                "/follow_stop")
            return

        if path == "/action":
            names = query.get("name", [])
            if not names:
                self.send_text(400, "missing action name")
                return
            ok, msg = write_action(names[0].strip())
            print("[ACTION]" if ok else "[ACTION_ERR]", msg)
            self.send_text(200 if ok else 400, msg)
            return

        if path == "/trigger":
            write_action("choki")
            self.send_text(200, "triggered")
            return

        if path == "/follow_stop":
            write_follow(False)
            write_action("follow_stop")
            print("[FOLLOW] stop")
            self.send_text(200, "follow stopped")
            return

        if path == "/follow":
            try:
                enable = int(query.get("enable", ["0"])[0]) != 0
                vx = float(query.get("vx", ["0"])[0])
                vy = float(query.get("vy", ["0"])[0])
                wz = float(query.get("wz", ["0"])[0])
            except Exception as e:
                self.send_text(400, "bad follow query: %s" % e)
                return

            write_follow(enable, vx, vy, wz)
            msg = "follow enable=%d vx=%.3f vy=%.3f wz=%.3f" % (1 if enable else 0, vx, vy, wz)
            print("[FOLLOW]", msg)
            self.send_text(200, msg)
            return

        if path == "/snapshot.jpg":
            with latest_lock:
                jpg = latest_jpeg
            if jpg is None:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b"no frame\n")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
            return

        if path == "/video":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with latest_lock:
                        jpg = latest_jpeg
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return

        self.send_response(404)
        self.end_headers()


def main():
    write_follow(False)

    t = threading.Thread(target=camera_loop)
    t.daemon = True
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("[INFO] server started")
    print("[INFO] status:   http://192.168.12.1:%d/status" % PORT)
    print("[INFO] video:    http://192.168.12.1:%d/video" % PORT)
    print("[INFO] follow:   http://192.168.12.1:%d/follow?enable=1&vx=0.10&vy=0&wz=0" % PORT)
    print("[INFO] stop:     http://192.168.12.1:%d/follow_stop" % PORT)
    server.serve_forever()


if __name__ == "__main__":
    main()
