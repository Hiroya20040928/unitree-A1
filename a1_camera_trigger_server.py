#!/usr/bin/env python3
import cv2
import time
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

CAM_ID = 2
PORT = 8090

TRIGGER_PATH = "/tmp/a1_choki_trigger"
ACTION_PATH = "/tmp/a1_action"

latest_jpeg = None
latest_lock = threading.Lock()


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


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

        ok, jpg = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )

        if ok:
            with latest_lock:
                latest_jpeg = jpg.tobytes()

        time.sleep(0.03)


def write_action(name):
    # Allow only safe action names.
    allowed = {
        "ready",
        "prone",
        "wave",
        "shake",
        "sway",
        "stop",
        "emergency_prone",
        "choki",
    }

    if name not in allowed:
        return False, "unknown action: " + name

    with open(ACTION_PATH, "w") as f:
        f.write(name + "\n")

    # Compatibility with the current completed A1 program.
    # choki/prone/emergency_prone also create the old trigger file.
    if name in ("choki", "prone", "emergency_prone"):
        with open(TRIGGER_PATH, "w") as f:
            f.write(name + "\n")

    return True, "action written: " + name


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        global latest_jpeg

        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        query = urllib.parse.parse_qs(parsed.query)

        if path == "/" or path == "/status":
            body = (
                "OK\n"
                "/video\n"
                "/snapshot.jpg\n"
                "/trigger\n"
                "/action?name=ready\n"
                "/action?name=prone\n"
                "/action?name=wave\n"
                "/action?name=shake\n"
                "/action?name=sway\n"
                "/action?name=stop\n"
                "/action?name=emergency_prone\n"
            )

            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(body.encode())
            return

        if path == "/trigger":
            with open(TRIGGER_PATH, "w") as f:
                f.write("choki\n")

            with open(ACTION_PATH, "w") as f:
                f.write("choki\n")

            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"triggered\n")
            print("[TRIGGER] wrote", TRIGGER_PATH)
            print("[ACTION] wrote", ACTION_PATH, "choki")
            return

        if path == "/action":
            names = query.get("name", [])

            if not names:
                self.send_response(400)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"missing action name\n")
                return

            name = names[0].strip()
            ok, msg = write_action(name)

            if ok:
                self.send_response(200)
                print("[ACTION]", msg)
            else:
                self.send_response(400)
                print("[ACTION_ERR]", msg)

            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write((msg + "\n").encode())
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

            except BrokenPipeError:
                return
            except ConnectionResetError:
                return

        self.send_response(404)
        self.end_headers()


def main():
    t = threading.Thread(target=camera_loop)
    t.daemon = True
    t.start()

    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)

    print("[INFO] server started")
    print("[INFO] status:   http://192.168.12.1:%d/status" % PORT)
    print("[INFO] video:    http://192.168.12.1:%d/video" % PORT)
    print("[INFO] snapshot: http://192.168.12.1:%d/snapshot.jpg" % PORT)
    print("[INFO] trigger:  http://192.168.12.1:%d/trigger" % PORT)
    print("[INFO] action:   http://192.168.12.1:%d/action?name=ready" % PORT)

    server.serve_forever()


if __name__ == "__main__":
    main()
