#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class A1ROSHandController:
    def __init__(self):
        # 1. ROSノード初期化
        rospy.init_node('a1_hand_control_ros_node', anonymous=True)
        
        # 2. 下半身への速度命令パブリッシャ
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_cmd = Twist()
        
        # 3. OpenCV ↔ ROS画像トピックの相互変換ブリッジ（Python 2完全直通）
        self.bridge = CvBridge()
        
        # 4. すでに配信されているRealSenseの画像トピックをサブスクライブ
        # ※もしトピック名が違う場合は、環境に合わせて /camera/rgb/image_raw 等に変更してください
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.last_motion_time = 0
        rospy.loginfo("==============================================")
        rospy.loginfo("ROS-Direct Hand Controller Initialized (Python 2)!")
        rospy.loginfo("Subscribed to /camera/color/image_raw")
        rospy.loginfo("==============================================")

    def count_fingers_native(self, frame):
        """
        OpenCVの幾何学解析を用いて肌色領域から指の数を算出
        """
        # 【環境適応】研究室の蛍光灯や夕方の光に対応するため、しきい値をワイドに拡張
        min_hsv = np.array([0, 15, 40], dtype="uint8")
        max_hsv = np.array([30, 255, 255], dtype="uint8")
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        skin_mask = cv2.inRange(hsv, min_hsv, max_hsv)
        
        blur = cv2.blur(skin_mask, (2, 2))
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return 0, frame

        max_contour = max(contours, key=lambda x: cv2.contourArea(x))
        if cv2.contourArea(max_contour) < 8000:
            return 0, frame

        hull = cv2.convexHull(max_contour, returnPoints=False)
        defects = cv2.convexDefects(max_contour, hull)
        
        finger_cnt = 0
        if defects is not None:
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = tuple(max_contour[s][0])
                end = tuple(max_contour[e][0])
                far = tuple(max_contour[f][0])
                
                a = np.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                b = np.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                c = np.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                
                angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c)) * 57
                
                if angle <= 90 and d > 10000:
                    finger_cnt += 1
                    cv2.circle(frame, far, 5, [0, 0, 255], -1)
                
                cv2.line(frame, start, end, [0, 255, 0], 2)

            if finger_cnt > 0:
                finger_cnt += 1

        return finger_cnt, frame

    def execute_twist_motion(self):
        rospy.loginfo("Choki (2) Detected! Executing body twist motion.")
        self.twist_cmd.linear.x = 0.0
        
        # 右ひねり
        self.twist_cmd.angular.z = 0.6
        t_end = time.time() + 0.4
        while time.time() < t_end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            time.sleep(0.05)

        # 左ひねり
        self.twist_cmd.angular.z = -0.6
        t_end = time.time() + 0.4
        while time.time() < t_end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist_cmd)
            time.sleep(0.05)

        # 復帰・停止
        self.twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_cmd)
        self.last_motion_time = time.time()

    def image_callback(self, ros_data):
        if time.time() - self.last_motion_time < 2.0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # ─── 【デバッグ用】現在の肌色マスクの抽出状態をPC画面に強制表示 ───
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        min_hsv = np.array([0, 15, 40], dtype="uint8")
        max_hsv = np.array([30, 255, 255], dtype="uint8")
        skin_mask = cv2.inRange(hsv, min_hsv, max_hsv)
        cv2.imshow("DEBUG: Skin Mask (White is Hand)", skin_mask)
        # ──────────────────────────────────────────────────────────────────

        # 指の本数を解析
        finger_count, debug_frame = self.count_fingers_native(frame)

        text = "Fingers: " + str(finger_count)
        cv2.putText(debug_frame, text, (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)

        if finger_count == 2:
            self.execute_twist_motion()

        cv2.imshow("A1 ROS Direct Window", debug_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        controller = A1ROSHandController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
