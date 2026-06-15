#!/usr/bin/env python3
import argparse
import time
import cv2
import numpy as np


def skin_mask_bgr(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    ycrcb = cv2.cvtColor(blur, cv2.COLOR_BGR2YCrCb)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Skin-color mask. Conservative enough for this test.
    mask_ycrcb = cv2.inRange(
        ycrcb,
        np.array([0, 135, 80]),
        np.array([255, 175, 130])
    )

    mask_hsv = cv2.inRange(
        hsv,
        np.array([0, 25, 45]),
        np.array([25, 255, 255])
    )

    mask = cv2.bitwise_and(mask_ycrcb, mask_hsv)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=1)

    return mask


def contour_touches_border(x, y, w, h, W, H, margin=12):
    if x <= margin:
        return True
    if y <= margin:
        return True
    if x + w >= W - margin:
        return True
    if y + h >= H - margin:
        return True
    return False


def analyze_contour(contour, image_w, image_h):
    area = cv2.contourArea(contour)

    if area < 9000:
        return False, 0, area, 0, "area_small"

    if area > 120000:
        return False, 0, area, 0, "area_large"

    x, y, w, h = cv2.boundingRect(contour)

    if contour_touches_border(x, y, w, h, image_w, image_h):
        return False, 0, area, 0, "touch_border"

    if w < 70 or h < 90:
        return False, 0, area, 0, "bbox_small"

    aspect = w / float(h)

    # A hand / peace sign in the selected ROI should not be extremely thin or wide.
    if aspect < 0.25 or aspect > 1.8:
        return False, 0, area, 0, "aspect_bad"

    hull_points = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull_points)

    if hull_area < 1:
        return False, 0, area, 0, "hull_bad"

    solidity = area / hull_area

    # Choki has gaps between fingers, so it should not be almost perfectly convex.
    # But too low solidity is often background noise.
    if solidity < 0.45 or solidity > 0.92:
        return False, 0, area, 0, "solidity_bad"

    hull_indices = cv2.convexHull(contour, returnPoints=False)

    if hull_indices is None or len(hull_indices) < 4:
        return False, 0, area, 0, "hull_indices_bad"

    defects = cv2.convexityDefects(contour, hull_indices)

    if defects is None:
        return False, 0, area, 0, "no_defects"

    valid_defects = 0

    for i in range(defects.shape[0]):
        s, e, f, depth = defects[i, 0]

        start = contour[s][0]
        end = contour[e][0]
        far = contour[f][0]

        a = np.linalg.norm(end - start)
        b = np.linalg.norm(far - start)
        c = np.linalg.norm(end - far)

        if b < 1e-6 or c < 1e-6:
            continue

        cos_angle = (b*b + c*c - a*a) / (2*b*c)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.degrees(np.arccos(cos_angle))
        depth_px = depth / 256.0

        # Stricter than previous version.
        if angle < 75 and depth_px > 35:
            valid_defects += 1

    fingers = valid_defects + 1 if valid_defects > 0 else 0

    if fingers > 5:
        fingers = 5

    # For this application, accept only clear 2-finger state.
    is_choki = (fingers == 2)

    reason = (
        f"ok" if is_choki else
        f"not_choki_fingers_{fingers}"
    )

    return is_choki, fingers, area, valid_defects, reason


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", type=int, default=2)
    parser.add_argument("--trigger", type=str, default="/tmp/a1_choki_trigger")
    parser.add_argument("--required", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--debug-image", type=str, default="/tmp/choki_debug.jpg")
    parser.add_argument("--debug-mask", type=str, default="/tmp/choki_mask.jpg")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.cam)

    if not cap.isOpened():
        print(f"[ERR] camera {args.cam} could not be opened")
        return 1

    print(f"[INFO] camera {args.cam} opened")
    print(f"[INFO] trigger path: {args.trigger}")
    print("[INFO] Put the hand in the CENTER of the image.")
    print("[INFO] Background should be simple and bright.")
    print("[INFO] Ctrl+C to stop")

    consecutive = 0
    frame_count = 0

    last_reason = "none"

    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("[WARN] no frame")
                time.sleep(0.1)
                continue

            frame_count += 1

            h, w = frame.shape[:2]

            # Narrow center ROI.
            # This intentionally excludes the right edge false-positive region.
            x1 = int(w * 0.32)
            x2 = int(w * 0.68)
            y1 = int(h * 0.08)
            y2 = int(h * 0.92)

            roi = frame[y1:y2, x1:x2]
            roi_small = cv2.resize(roi, (640, 480))

            mask = skin_mask_bgr(roi_small)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            fingers = 0
            area = 0
            defects = 0
            is_choki = False
            reason = "no_contour"

            debug = roi_small.copy()

            if contours:
                # Sort by area and inspect largest candidates.
                contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)

                for contour in contours_sorted[:3]:
                    ok, f, a, d, r = analyze_contour(contour, 640, 480)

                    x, y, bw, bh = cv2.boundingRect(contour)
                    color = (0, 255, 0) if ok else (0, 0, 255)
                    cv2.drawContours(debug, [contour], -1, color, 2)
                    cv2.rectangle(debug, (x, y), (x + bw, y + bh), color, 2)

                    if ok:
                        is_choki = True
                        fingers = f
                        area = a
                        defects = d
                        reason = r
                        break

                    if a > area:
                        fingers = f
                        area = a
                        defects = d
                        reason = r

            last_reason = reason

            if is_choki:
                consecutive += 1
            else:
                consecutive = 0

            if frame_count % 5 == 0:
                print(
                    f"[CHOKI] ok={int(is_choki)} fingers={fingers} defects={defects} "
                    f"area={area:.0f} consecutive={consecutive}/{args.required} reason={reason}"
                )

            if frame_count % 10 == 0:
                cv2.imwrite(args.debug_image, debug)
                cv2.imwrite(args.debug_mask, mask)

            if consecutive >= args.required:
                print("[TRIGGER] choki detected")

                if not args.dry_run:
                    with open(args.trigger, "w") as f:
                        f.write("choki\n")
                    print(f"[TRIGGER] wrote {args.trigger}")
                else:
                    print("[DRY-RUN] trigger not written")

                break

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] stopped")

    finally:
        cap.release()
        print(f"[INFO] last reason: {last_reason}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
