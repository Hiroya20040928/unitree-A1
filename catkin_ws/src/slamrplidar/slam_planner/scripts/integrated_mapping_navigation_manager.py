#!/usr/bin/env python
# coding: utf-8

from __future__ import print_function

import io
import os
import subprocess
import threading
import time
from datetime import datetime

import rospy
import tf
import yaml
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from slamware_ros_sdk.msg import ClearMapRequest
from slamware_ros_sdk.msg import MapKind
from slamware_ros_sdk.msg import SetMapLocalizationRequest
from slamware_ros_sdk.msg import SetMapUpdateRequest
from slamware_ros_sdk.msg import SyncMapRequest
from slamware_ros_sdk.srv import SyncGetStcm
from slamware_ros_sdk.srv import SyncSetStcm
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class IntegratedMappingNavigationManager(object):
    def __init__(self):
        self.bundle_root = os.path.expanduser(
            rospy.get_param("~bundle_root", "~/catkin_ws/src/slamrplidar/slam_planner/maps/generated")
        )
        self.map_bundle = rospy.get_param("~map_bundle", "").strip()
        self.save_map_name = rospy.get_param("~save_map_name", "").strip()
        self.start_fresh_map = rospy.get_param("~start_fresh_map", True)
        self.auto_start_navigation_from_bundle = rospy.get_param(
            "~auto_start_navigation_from_bundle", True
        )
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.sync_wait_sec = float(rospy.get_param("~sync_wait_sec", 1.5))
        self.dependency_poll_sec = float(rospy.get_param("~dependency_poll_sec", 1.0))
        self.dependency_log_sec = float(rospy.get_param("~dependency_log_sec", 5.0))
        self.navigation_launch = rospy.get_param(
            "~navigation_launch", "roslaunch slam_planner integrated_navigation_stack.launch"
        )
        self.server_prefix = rospy.get_param(
            "~server_prefix", "/slamware_ros_sdk_server_node"
        ).rstrip("/")

        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        self.sync_map_pub = rospy.Publisher(
            self.server_prefix + "/sync_map", SyncMapRequest, queue_size=1
        )
        self.clear_map_pub = rospy.Publisher(
            self.server_prefix + "/clear_map", ClearMapRequest, queue_size=1
        )
        self.map_update_pub = rospy.Publisher(
            self.server_prefix + "/set_map_update", SetMapUpdateRequest, queue_size=1
        )
        self.map_localization_pub = rospy.Publisher(
            self.server_prefix + "/set_map_localization",
            SetMapLocalizationRequest,
            queue_size=1,
        )

        self.finish_service = rospy.Service(
            "~finish_mapping", Trigger, self.handle_finish_mapping
        )

        self.sync_get_stcm = rospy.ServiceProxy(
            self.server_prefix + "/sync_get_stcm", SyncGetStcm
        )
        self.sync_set_stcm = rospy.ServiceProxy(
            self.server_prefix + "/sync_set_stcm", SyncSetStcm
        )

        self.last_map_time = None
        self.last_scan_time = None
        self.last_odom_time = None
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.handle_map, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.handle_scan, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.handle_odom, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.navigation_process = None
        self.mode_lock = threading.Lock()
        self.mode = "initializing"
        self.set_mode("initializing")

        rospy.on_shutdown(self.shutdown)

    def run(self):
        self.wait_for_dependencies()
        rospy.sleep(1.0)

        if self.map_bundle:
            self.load_bundle(self.map_bundle)
            if self.auto_start_navigation_from_bundle:
                self.start_navigation()
            self.set_mode("navigation")
            rospy.loginfo("Loaded map bundle: %s", self.map_bundle)
        else:
            if self.start_fresh_map:
                self.reset_mapping_session()
            self.set_mode("mapping")
            rospy.loginfo("Mapping mode started. Use the official remote controller to walk the robot.")

        rospy.spin()

    def wait_for_dependencies(self):
        self.set_mode("waiting_for_live_slamware")
        rospy.loginfo(
            "Waiting for live SLAMWare data on %s and %s, plus STCM services...",
            self.map_topic,
            self.scan_topic,
        )

        next_log_time = 0.0
        next_sync_time = 0.0

        while not rospy.is_shutdown():
            services_ready = self.check_required_services()
            map_ready = self.last_map_time is not None
            scan_ready = self.last_scan_time is not None

            if services_ready and map_ready and scan_ready:
                break

            now = time.time()
            if services_ready and now >= next_sync_time:
                self.request_sync_map()
                next_sync_time = now + max(self.sync_wait_sec, 1.0)

            if now >= next_log_time:
                rospy.logwarn(
                    "Still waiting for live SLAMWare data: services=%s map=%s scan=%s odom=%s",
                    services_ready,
                    map_ready,
                    scan_ready,
                    self.last_odom_time is not None,
                )
                next_log_time = now + max(self.dependency_log_sec, 1.0)

            rospy.sleep(self.dependency_poll_sec)

        if rospy.is_shutdown():
            raise rospy.ROSInterruptException("Interrupted while waiting for SLAMWare data.")

        rospy.loginfo("Dependencies are ready.")

    def check_required_services(self):
        try:
            self.sync_get_stcm.wait_for_service(timeout=0.2)
            self.sync_set_stcm.wait_for_service(timeout=0.2)
            return True
        except rospy.ROSException:
            return False

    def handle_map(self, _msg):
        self.last_map_time = time.time()

    def handle_scan(self, _msg):
        self.last_scan_time = time.time()

    def handle_odom(self, _msg):
        self.last_odom_time = time.time()

    def reset_mapping_session(self):
        clear_msg = ClearMapRequest()
        clear_msg.kind.kind = MapKind.SLAMMAP
        self.clear_map_pub.publish(clear_msg)
        rospy.sleep(0.5)
        clear_msg.kind.kind = MapKind.LOCALSLAMMAP
        self.clear_map_pub.publish(clear_msg)
        rospy.sleep(0.5)
        self.set_map_update(True, MapKind.SLAMMAP)
        self.set_map_update(True, MapKind.LOCALSLAMMAP)
        self.set_map_localization(True)
        self.request_sync_map()

    def handle_finish_mapping(self, _req):
        with self.mode_lock:
            if self.mode != "mapping":
                return TriggerResponse(
                    success=False,
                    message="The manager is not in mapping mode.",
                )

            try:
                self.freeze_mapping()
                bundle_dir = self.save_bundle()
                self.start_navigation()
                self.set_mode("navigation")
                message = "Saved bundle to {0} and started navigation.".format(bundle_dir)
                rospy.loginfo(message)
                return TriggerResponse(success=True, message=message)
            except Exception as exc:
                rospy.logerr("Failed to finish mapping: %s", exc)
                return TriggerResponse(success=False, message=str(exc))

    def freeze_mapping(self):
        self.request_sync_map()
        rospy.sleep(self.sync_wait_sec)
        self.set_map_update(False, MapKind.SLAMMAP)
        self.set_map_update(False, MapKind.LOCALSLAMMAP)
        self.set_map_localization(True)
        rospy.sleep(0.5)

    def save_bundle(self):
        bundle_dir = self.prepare_bundle_dir()
        pose = self.lookup_robot_pose()

        sync_resp = self.sync_get_stcm()
        stcm_path = os.path.join(bundle_dir, "map.stcm")
        with io.open(stcm_path, "wb") as handle:
            handle.write(bytearray(sync_resp.raw_stcm))

        map_base_path = os.path.join(bundle_dir, "map")
        map_saver_cmd = [
            "rosrun",
            "map_server",
            "map_saver",
            "-f",
            map_base_path,
            "map:={0}".format(self.map_topic),
        ]
        rospy.loginfo("Saving 2D occupancy map with: %s", " ".join(map_saver_cmd))
        subprocess.check_call(map_saver_cmd)

        metadata = {
            "bundle_version": 1,
            "created_at_utc": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
            "map_frame": self.map_frame,
            "base_frame": self.base_frame,
            "files": {
                "stcm": "map.stcm",
                "yaml": "map.yaml",
                "image": "map.pgm",
            },
            "pose": {
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                },
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w,
                },
            },
        }
        metadata_path = os.path.join(bundle_dir, "metadata.yaml")
        with io.open(metadata_path, "w", encoding="utf-8") as handle:
            yaml.safe_dump(metadata, handle, default_flow_style=False)

        rospy.set_param("~last_saved_bundle", bundle_dir)
        return bundle_dir

    def load_bundle(self, bundle_path):
        bundle_dir, stcm_path, metadata_path = self.resolve_bundle_paths(bundle_path)

        with io.open(stcm_path, "rb") as handle:
            raw_stcm = list(bytearray(handle.read()))

        pose = Pose()
        if os.path.exists(metadata_path):
            with io.open(metadata_path, "r", encoding="utf-8") as handle:
                metadata = yaml.safe_load(handle) or {}
            pose_data = metadata.get("pose", {})
            pos = pose_data.get("position", {})
            ori = pose_data.get("orientation", {})
            pose.position.x = float(pos.get("x", 0.0))
            pose.position.y = float(pos.get("y", 0.0))
            pose.position.z = float(pos.get("z", 0.0))
            pose.orientation.x = float(ori.get("x", 0.0))
            pose.orientation.y = float(ori.get("y", 0.0))
            pose.orientation.z = float(ori.get("z", 0.0))
            pose.orientation.w = float(ori.get("w", 1.0))
        else:
            pose.orientation.w = 1.0
            rospy.logwarn("No metadata file found at %s. Using identity pose.", metadata_path)

        req = self.sync_set_stcm._request_class()
        req.raw_stcm = raw_stcm
        req.robot_pose = pose
        self.sync_set_stcm(req)

        self.publish_initial_pose(pose)
        self.set_map_update(False, MapKind.SLAMMAP)
        self.set_map_update(False, MapKind.LOCALSLAMMAP)
        self.set_map_localization(True)
        self.request_sync_map()
        rospy.set_param("~last_loaded_bundle", bundle_dir)

    def resolve_bundle_paths(self, bundle_path):
        resolved = os.path.expanduser(bundle_path)
        if os.path.isdir(resolved):
            bundle_dir = resolved
            stcm_path = os.path.join(bundle_dir, "map.stcm")
        else:
            bundle_dir = os.path.dirname(resolved)
            stcm_path = resolved

        metadata_path = os.path.join(bundle_dir, "metadata.yaml")

        if not os.path.exists(stcm_path):
            raise IOError("Map bundle does not contain map.stcm: {0}".format(stcm_path))

        return bundle_dir, stcm_path, metadata_path

    def start_navigation(self):
        if self.navigation_process is not None:
            if self.navigation_process.poll() is None:
                return
            self.navigation_process = None

        cmd = self.navigation_launch.split()
        rospy.loginfo("Starting navigation stack: %s", " ".join(cmd))
        self.navigation_process = subprocess.Popen(cmd)
        rospy.sleep(2.0)

    def lookup_robot_pose(self):
        self.tf_listener.waitForTransform(
            self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(5.0)
        )
        trans, rot = self.tf_listener.lookupTransform(
            self.map_frame, self.base_frame, rospy.Time(0)
        )
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return pose

    def publish_initial_pose(self, pose):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = self.map_frame
        initial_pose.pose.pose = pose
        initial_pose.pose.covariance[0] = 0.05
        initial_pose.pose.covariance[7] = 0.05
        initial_pose.pose.covariance[35] = 0.1
        self.initial_pose_pub.publish(initial_pose)

    def prepare_bundle_dir(self):
        root_dir = os.path.expanduser(self.bundle_root)
        if not os.path.isdir(root_dir):
            os.makedirs(root_dir)

        if self.save_map_name:
            candidate = os.path.join(root_dir, self.save_map_name)
            if not os.path.exists(candidate):
                os.makedirs(candidate)
                return candidate
            timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            candidate = os.path.join(root_dir, "{0}_{1}".format(self.save_map_name, timestamp))
            os.makedirs(candidate)
            return candidate

        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        candidate = os.path.join(root_dir, timestamp)
        os.makedirs(candidate)
        return candidate

    def request_sync_map(self):
        self.sync_map_pub.publish(SyncMapRequest())

    def set_map_update(self, enabled, kind_value):
        msg = SetMapUpdateRequest()
        msg.enabled = enabled
        msg.kind.kind = kind_value
        self.map_update_pub.publish(msg)

    def set_map_localization(self, enabled):
        msg = SetMapLocalizationRequest()
        msg.enabled = enabled
        self.map_localization_pub.publish(msg)

    def set_mode(self, mode):
        self.mode = mode
        self.status_pub.publish(String(data=mode))
        rospy.set_param("~mode", mode)

    def shutdown(self):
        if self.navigation_process is not None and self.navigation_process.poll() is None:
            self.navigation_process.terminate()
            try:
                self.navigation_process.wait(5)
            except Exception:
                self.navigation_process.kill()


if __name__ == "__main__":
    rospy.init_node("integrated_mapping_navigation")
    manager = IntegratedMappingNavigationManager()
    manager.run()
