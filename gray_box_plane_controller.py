#!/usr/bin/env python3
"""Servo to a gray box using a RANSAC plane fitted to the masked point cloud."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Twist
from motor_move_msgs.action import MotorMove, MoveToShelf
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField


@dataclass
class PlaneFit:
    normal: np.ndarray
    centroid: np.ndarray
    inlier_mask: np.ndarray
    rms_m: float
    width_m: float
    height_m: float


@dataclass
class ClusterStats:
    points: np.ndarray
    centroid: np.ndarray
    left_mean: np.ndarray
    right_mean: np.ndarray
    left_count: int
    right_count: int
    left_right_depth_delta_m: float
    width_m: float
    height_m: float
    nearest_depth_m: float


class GrayBoxPlaneController(Node):
    def __init__(self) -> None:
        super().__init__("gray_box_plane_controller")

        self.declare_parameter("image_topic", "/camera/frame_rgb")
        self.declare_parameter("pointcloud_topic", "/camera/frame_pc")
        self.declare_parameter("cmd_vel_topic", "/robotinobase1/cmd_vel")
        self.declare_parameter("motor_action", "/robotinobase1/motor_move_action")
        self.declare_parameter("align_action", "/robotinobase1/gray_box_align_action")
        self.declare_parameter("base_frame", "robotinobase1/base_link")
        self.declare_parameter("target_frame", "robotinobase1/gray_box_motor_target")
        self.declare_parameter("output_path", "/tmp/gray_box_plane_controller.png")
        self.declare_parameter("topdown_output_path", "/tmp/gray_box_plane_topdown.png")
        self.declare_parameter("show_gui", True)
        self.declare_parameter("window_name", "gray box plane controller")
        self.declare_parameter("topdown_window_name", "gray box top down")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("max_image_age_sec", 0.25)
        self.declare_parameter("max_cloud_age_sec", 0.35)
        self.declare_parameter("max_future_stamp_sec", 0.20)

        self.declare_parameter("gray_saturation_max", 70)
        self.declare_parameter("gray_value_min", 25)
        self.declare_parameter("gray_value_max", 95)
        self.declare_parameter("use_cloud_depth_mask", True)
        self.declare_parameter("depth_mask_dilate_px", 3)
        self.declare_parameter("center_seed_fraction", 0.18)
        self.declare_parameter("min_area_px", 1000)
        self.declare_parameter("max_area_fraction", 0.70)
        self.declare_parameter("min_aspect_ratio", 1.25)
        self.declare_parameter("tracking_memory_weight", 0.75)

        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 1.00)
        self.declare_parameter("max_points", 2500)
        self.declare_parameter("mask_dilate_px", 5)
        self.declare_parameter("ransac_iterations", 90)
        self.declare_parameter("ransac_threshold_m", 0.012)
        self.declare_parameter("min_inliers", 30)
        self.declare_parameter("max_plane_rms_m", 0.020)
        self.declare_parameter("filter_alpha", 0.35)

        self.declare_parameter("target_distance_m", 0.10)
        self.declare_parameter("approach_speed", 0.10)
        self.declare_parameter("motor_linear_speed", 0.10)
        self.declare_parameter("motor_angular_speed", 0.30)
        self.declare_parameter("max_motor_step_dt_sec", 0.25)
        self.declare_parameter("yaw_deadband_deg", 3.0)
        self.declare_parameter("lateral_deadband_m", 0.005)
        self.declare_parameter("distance_deadband_m", 0.025)
        self.declare_parameter("max_angular_speed", 0.10)
        self.declare_parameter("max_lateral_speed", 0.10)
        self.declare_parameter("max_forward_speed", 0.10)
        self.declare_parameter("cmd_republish_period_sec", 0.05)
        self.declare_parameter("side_band_fraction", 0.20)
        self.declare_parameter("min_side_points", 8)
        self.declare_parameter("cloud_tracking_enabled", False)
        self.declare_parameter("use_motor_move", False)
        self.declare_parameter("motor_goal_period_sec", 0.0)
        self.declare_parameter("motor_result_wait_sec", 0.0)
        self.declare_parameter("position_average_frames", 5)
        self.declare_parameter("require_fresh_image_after_motion", False)
        self.declare_parameter("track_radius_m", 0.18)
        self.declare_parameter("track_depth_radius_m", 0.25)
        self.declare_parameter("lost_stop_after_sec", 0.70)
        self.declare_parameter("yaw_depth_deadband_m", 0.010)
        self.declare_parameter("yaw_depth_kp", 1.2)
        self.declare_parameter("yaw_kp", 1.8)
        self.declare_parameter("lateral_kp", 0.90)
        self.declare_parameter("forward_kp", 0.45)
        self.declare_parameter("invert_angular", True)
        self.declare_parameter("invert_lateral", False)
        self.declare_parameter("action_enables_motion", True)
        self.declare_parameter("centered_stable_frames", 2)
        self.declare_parameter("run_only_during_action", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.motor_action = str(self.get_parameter("motor_action").value)
        self.align_action = str(self.get_parameter("align_action").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.topdown_output_path = str(self.get_parameter("topdown_output_path").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.topdown_window_name = str(self.get_parameter("topdown_window_name").value)

        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.max_image_age_sec = float(self.get_parameter("max_image_age_sec").value)
        self.max_cloud_age_sec = float(self.get_parameter("max_cloud_age_sec").value)
        self.max_future_stamp_sec = float(self.get_parameter("max_future_stamp_sec").value)

        self.gray_saturation_max = int(self.get_parameter("gray_saturation_max").value)
        self.gray_value_min = int(self.get_parameter("gray_value_min").value)
        self.gray_value_max = int(self.get_parameter("gray_value_max").value)
        self.use_cloud_depth_mask = bool(self.get_parameter("use_cloud_depth_mask").value)
        self.depth_mask_dilate_px = int(self.get_parameter("depth_mask_dilate_px").value)
        self.center_seed_fraction = float(self.get_parameter("center_seed_fraction").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)
        self.max_area_fraction = float(self.get_parameter("max_area_fraction").value)
        self.min_aspect_ratio = float(self.get_parameter("min_aspect_ratio").value)
        self.tracking_memory_weight = float(self.get_parameter("tracking_memory_weight").value)

        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.mask_dilate_px = int(self.get_parameter("mask_dilate_px").value)
        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.ransac_threshold_m = float(self.get_parameter("ransac_threshold_m").value)
        self.min_inliers = int(self.get_parameter("min_inliers").value)
        self.max_plane_rms_m = float(self.get_parameter("max_plane_rms_m").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)

        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.approach_speed = abs(float(self.get_parameter("approach_speed").value))
        self.motor_linear_speed = abs(float(self.get_parameter("motor_linear_speed").value))
        self.motor_angular_speed = abs(float(self.get_parameter("motor_angular_speed").value))
        self.max_motor_step_dt_sec = max(0.01, float(self.get_parameter("max_motor_step_dt_sec").value))
        self.yaw_deadband_rad = math.radians(float(self.get_parameter("yaw_deadband_deg").value))
        self.lateral_deadband_m = float(self.get_parameter("lateral_deadband_m").value)
        self.distance_deadband_m = float(self.get_parameter("distance_deadband_m").value)
        self.max_angular_speed = abs(float(self.get_parameter("max_angular_speed").value))
        self.max_lateral_speed = abs(float(self.get_parameter("max_lateral_speed").value))
        self.max_forward_speed = abs(float(self.get_parameter("max_forward_speed").value))
        self.cmd_republish_period_sec = max(0.01, float(self.get_parameter("cmd_republish_period_sec").value))
        self.side_band_fraction = float(self.get_parameter("side_band_fraction").value)
        self.min_side_points = int(self.get_parameter("min_side_points").value)
        self.cloud_tracking_enabled = bool(self.get_parameter("cloud_tracking_enabled").value)
        self.use_motor_move = bool(self.get_parameter("use_motor_move").value)
        self.motor_goal_period_sec = float(self.get_parameter("motor_goal_period_sec").value)
        self.motor_result_wait_sec = max(0.0, float(self.get_parameter("motor_result_wait_sec").value))
        self.position_average_frames = max(1, int(self.get_parameter("position_average_frames").value))
        self.require_fresh_image_after_motion = bool(self.get_parameter("require_fresh_image_after_motion").value)
        self.track_radius_m = float(self.get_parameter("track_radius_m").value)
        self.track_depth_radius_m = float(self.get_parameter("track_depth_radius_m").value)
        self.lost_stop_after_sec = float(self.get_parameter("lost_stop_after_sec").value)
        self.yaw_depth_deadband_m = float(self.get_parameter("yaw_depth_deadband_m").value)
        self.yaw_depth_kp = float(self.get_parameter("yaw_depth_kp").value)
        self.yaw_kp = float(self.get_parameter("yaw_kp").value)
        self.lateral_kp = float(self.get_parameter("lateral_kp").value)
        self.forward_kp = float(self.get_parameter("forward_kp").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)
        self.invert_lateral = bool(self.get_parameter("invert_lateral").value)
        self.action_enables_motion = bool(self.get_parameter("action_enables_motion").value)
        self.centered_stable_frames = max(1, int(self.get_parameter("centered_stable_frames").value))
        self.run_only_during_action = bool(self.get_parameter("run_only_during_action").value)

        self.bridge = CvBridge()
        self.latest_cloud: Optional[PointCloud2] = None
        self.latest_cloud_stamp_ns: Optional[int] = None
        self.last_processed_image_stamp_ns: Optional[int] = None
        self.latest_image: Optional[np.ndarray] = None
        self.latest_detection: Optional[tuple] = None
        self.latest_object_mask: Optional[np.ndarray] = None
        self.last_center_px: Optional[np.ndarray] = None
        self.last_cluster_centroid: Optional[np.ndarray] = None
        self.last_cluster_stamp_ns: Optional[int] = None
        self.filtered_yaw: Optional[float] = None
        self.filtered_lateral: Optional[float] = None
        self.filtered_distance: Optional[float] = None
        self.last_motor_goal_time_ns: Optional[int] = None
        self.last_motor_step_frame_ns: Optional[int] = None
        self.motor_goal_pending = False
        self.motor_active = False
        self.await_fresh_image_after_motion = False
        self.motor_wait_until_ns: Optional[int] = None
        self.latest_base_goal: Optional[PoseStamped] = None
        self.latest_topdown_object_xy: Optional[np.ndarray] = None
        self.latest_topdown_target_xy: Optional[np.ndarray] = None
        self.latest_topdown_target_yaw = 0.0
        self.pending_move_start_object_xy: Optional[np.ndarray] = None
        self.pending_move_target_xy: Optional[np.ndarray] = None
        self.pending_move_target_yaw = 0.0
        self.last_move_start_object_xy: Optional[np.ndarray] = None
        self.last_move_target_xy: Optional[np.ndarray] = None
        self.last_move_target_yaw = 0.0
        self.last_move_end_object_xy: Optional[np.ndarray] = None
        self.active_cmd = Twist()
        self.active_cmd_stamp_ns: Optional[int] = None
        self.cluster_history = deque(maxlen=self.position_average_frames)
        self.align_goal_handle = None
        self.align_started_ns: Optional[int] = None
        self.align_centered_count = 0
        self.align_latest_front_range = float("nan")
        self.frame_count = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.action_client = ActionClient(self, MotorMove, self.motor_action)
        self.align_action_server = ActionServer(
            self,
            MoveToShelf,
            self.align_action,
            goal_callback=self.on_align_goal,
            cancel_callback=self.on_align_cancel,
            handle_accepted_callback=self.on_align_accepted,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.image_topic, self.on_image, sensor_qos)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_cloud, sensor_qos)
        self.create_timer(0.1, self.on_align_timer)
        self.create_timer(self.cmd_republish_period_sec, self.on_cmd_timer)

        if self.show_gui:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 540)
            cv2.namedWindow(self.topdown_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.topdown_window_name, 640, 640)
            topdown = self.draw_topdown_debug(None, None, None, None, "waiting", None)
            cv2.imshow(self.topdown_window_name, topdown)
            cv2.imwrite(self.topdown_output_path, topdown)
            cv2.waitKey(1)

        self.get_logger().info(f"image: {self.image_topic}")
        self.get_logger().info(f"pointcloud: {self.pointcloud_topic}")
        self.get_logger().info(f"motor_move: {self.motor_action}, relative goals in {self.base_frame}")
        self.get_logger().info(f"gray box align action: {self.align_action}")
        self.get_logger().warn(f"motion is {'ENABLED' if self.enable_motion else 'disabled'}")

    def motion_enabled(self) -> bool:
        return self.enable_motion or (self.action_enables_motion and self.align_goal_active())

    def controller_active(self) -> bool:
        return not self.run_only_during_action or self.align_goal_active()

    def align_goal_active(self) -> bool:
        return self.align_goal_handle is not None and self.align_goal_handle.is_active

    def on_align_goal(self, goal_request: MoveToShelf.Goal) -> GoalResponse:
        if self.align_goal_active():
            self.get_logger().warn("rejecting gray box align goal: another goal is active")
            return GoalResponse.REJECT
        if goal_request.timeout <= 0.0:
            self.get_logger().warn("rejecting gray box align goal: timeout must be > 0")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_align_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("canceling gray box align goal")
        return CancelResponse.ACCEPT

    def on_align_accepted(self, goal_handle) -> None:
        self.align_goal_handle = goal_handle
        self.align_started_ns = self.get_clock().now().nanoseconds
        self.align_centered_count = 0
        self.cluster_history.clear()
        self.last_processed_image_stamp_ns = None
        self.last_motor_step_frame_ns = None
        self.filtered_yaw = None
        self.filtered_lateral = None
        self.filtered_distance = None
        goal_handle.executing()
        self.get_logger().info(
            f"accepted gray box align goal: timeout={goal_handle.request.timeout:.3f}s"
        )

    def finish_align_goal(self, success: bool, message: str, canceled: bool = False) -> None:
        goal_handle = self.align_goal_handle
        if goal_handle is None:
            return
        result = MoveToShelf.Result()
        result.success = success
        result.message = message
        if canceled:
            goal_handle.canceled(result)
        elif success:
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)
        self.align_goal_handle = None
        self.align_started_ns = None
        self.align_centered_count = 0
        self.latest_base_goal = None
        self.stop()
        self.get_logger().info(f"gray box align finished: success={success} {message}")

    def publish_align_feedback(self) -> None:
        if not self.align_goal_active() or self.align_started_ns is None:
            return
        feedback = MoveToShelf.Feedback()
        feedback.front_range = float(self.align_latest_front_range)
        feedback.elapsed_time = float(
            (self.get_clock().now().nanoseconds - self.align_started_ns) / 1_000_000_000.0
        )
        self.align_goal_handle.publish_feedback(feedback)

    def on_align_timer(self) -> None:
        if self.show_gui:
            cv2.waitKey(1)
        if not self.align_goal_active() or self.align_started_ns is None:
            return
        if self.align_goal_handle.is_cancel_requested:
            self.finish_align_goal(False, "gray box align canceled", canceled=True)
            return
        elapsed = (self.get_clock().now().nanoseconds - self.align_started_ns) / 1_000_000_000.0
        timeout = float(self.align_goal_handle.request.timeout)
        if elapsed > timeout:
            self.finish_align_goal(False, f"gray box align timed out after {elapsed:.1f}s")
            return
        self.publish_align_feedback()

    def close(self) -> None:
        if self.show_gui:
            cv2.destroyWindow(self.window_name)
            cv2.destroyWindow(self.topdown_window_name)

    def set_active_cmd(self, cmd: Twist) -> None:
        self.active_cmd = cmd
        self.active_cmd_stamp_ns = self.get_clock().now().nanoseconds

    def active_cmd_stale(self) -> bool:
        if self.active_cmd_stamp_ns is None:
            return True
        age = (self.get_clock().now().nanoseconds - self.active_cmd_stamp_ns) / 1_000_000_000.0
        return age > self.lost_stop_after_sec

    def on_cmd_timer(self) -> None:
        if self.use_motor_move or not self.motion_enabled() or not self.controller_active():
            return
        if self.active_cmd_stale():
            self.active_cmd = Twist()
            return
        self.cmd_pub.publish(self.active_cmd)

    def stop(self) -> None:
        self.active_cmd = Twist()
        self.active_cmd_stamp_ns = None
        if rclpy.ok():
            self.cmd_pub.publish(Twist())

    def stamp_to_ns(self, msg) -> Optional[int]:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def stamp_age_sec(self, msg) -> Optional[float]:
        stamp_ns = self.stamp_to_ns(msg)
        if stamp_ns is None:
            return None
        return (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0

    def too_old(self, msg, max_age_sec: float, name: str) -> bool:
        age = self.stamp_age_sec(msg)
        if age is None:
            return False
        if age > max_age_sec or age < -self.max_future_stamp_sec:
            self.get_logger().warn(
                f"dropping {name}: age={age:.3f}s",
                throttle_duration_sec=1.0,
            )
            return True
        return False

    def cloud_fresh(self) -> bool:
        if self.latest_cloud is None or self.latest_cloud_stamp_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - self.latest_cloud_stamp_ns) / 1_000_000_000.0
        return -self.max_future_stamp_sec <= age <= self.max_cloud_age_sec

    def on_cloud(self, msg: PointCloud2) -> None:
        if not self.controller_active():
            return
        if self.too_old(msg, self.max_cloud_age_sec, "pointcloud"):
            return
        self.latest_cloud = msg
        self.latest_cloud_stamp_ns = self.stamp_to_ns(msg) or self.get_clock().now().nanoseconds
        if self.cloud_tracking_enabled and not self.use_motor_move:
            self.control_from_latest_cloud()

    def image_from_msg(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding in ("bgr8", "rgb8"):
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if msg.encoding == "bgra8":
            return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8"), cv2.COLOR_BGRA2BGR)
        if msg.encoding == "rgba8":
            return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgba8"), cv2.COLOR_RGBA2BGR)
        self.get_logger().warn(f"unsupported image encoding {msg.encoding!r}")
        return None

    def cloud_depth_mask_for_image(self, image_shape: tuple[int, int]) -> Optional[np.ndarray]:
        if not self.use_cloud_depth_mask or not self.cloud_fresh():
            return None
        cloud = self.latest_cloud
        assert cloud is not None
        points = self.cloud_xyz(cloud)
        if points is None:
            return None

        cloud_mask = (
            np.isfinite(points).all(axis=2)
            & (points[:, :, 2] >= self.min_depth_m)
            & (points[:, :, 2] <= self.max_depth_m)
        )
        if not np.any(cloud_mask):
            return None

        image_h, image_w = image_shape
        image_mask = cv2.resize(
            cloud_mask.astype(np.uint8),
            (image_w, image_h),
            interpolation=cv2.INTER_NEAREST,
        )
        if self.depth_mask_dilate_px > 0:
            kernel_size = self.depth_mask_dilate_px * 2 + 1
            image_mask = cv2.dilate(image_mask, np.ones((kernel_size, kernel_size), dtype=np.uint8))
        return image_mask > 0

    def detect_gray_object(self, image: np.ndarray) -> tuple[Optional[tuple], np.ndarray, Optional[np.ndarray]]:
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray_mask_full = cv2.inRange(
            hsv,
            np.array([0, 0, self.gray_value_min], dtype=np.uint8),
            np.array([179, self.gray_saturation_max, self.gray_value_max], dtype=np.uint8),
        )
        depth_mask = self.cloud_depth_mask_for_image((height, width))
        if depth_mask is not None:
            gray_mask_full = cv2.bitwise_and(gray_mask_full, (depth_mask.astype(np.uint8) * 255))

        seed_w = int(width * self.center_seed_fraction)
        seed_h = int(height * self.center_seed_fraction)
        seed_x = (width - seed_w) // 2
        seed_y = (height - seed_h) // 2
        seed_mask = np.zeros((height, width), dtype=np.uint8)
        seed_mask[seed_y : seed_y + seed_h, seed_x : seed_x + seed_w] = 255

        count, labels, _, _ = cv2.connectedComponentsWithStats(gray_mask_full, connectivity=8)
        selected = np.zeros_like(gray_mask_full)
        seed_labels = np.unique(labels[(gray_mask_full > 0) & (seed_mask > 0)])
        for label_idx in seed_labels:
            if label_idx != 0:
                selected[labels == label_idx] = 255
        if not np.any(selected):
            selected = gray_mask_full

        selected = cv2.medianBlur(selected, 5)
        selected = cv2.morphologyEx(selected, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))
        contours, _ = cv2.findContours(selected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        image_center = np.array([width * 0.5, height * 0.5], dtype=np.float32)
        best = None
        best_score = float("inf")
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area_px or area > width * height * self.max_area_fraction:
                continue
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), angle = rect
            if rw <= 1.0 or rh <= 1.0:
                continue
            fill = area / max(1.0, rw * rh)
            aspect = max(rw, rh) / max(1.0, min(rw, rh))
            if fill < 0.25 or aspect < self.min_aspect_ratio or aspect > 8.0:
                continue

            candidate_center = np.array([cx, cy], dtype=np.float32)
            center_score = float(np.linalg.norm(candidate_center - image_center))
            if self.last_center_px is not None:
                previous_score = float(np.linalg.norm(candidate_center - self.last_center_px))
                w = float(np.clip(self.tracking_memory_weight, 0.0, 1.0))
                score = w * previous_score + (1.0 - w) * center_score
            else:
                score = center_score
            if score < best_score:
                best_score = score
                box = cv2.boxPoints(rect).astype(np.int32)
                best = (contour, box, cx, cy, rw, rh, angle, area, fill, aspect)

        if best is None:
            self.last_center_px = None
        else:
            _, _, cx, cy, *_ = best
            self.last_center_px = np.array([cx, cy], dtype=np.float32)
        return best, selected > 0, depth_mask

    def point_fields(self, cloud: PointCloud2) -> Optional[tuple[PointField, PointField, PointField]]:
        fields = {field.name: field for field in cloud.fields}
        x_field = fields.get("x")
        y_field = fields.get("y")
        z_field = fields.get("z")
        if not x_field or not y_field or not z_field:
            return None
        if any(field.datatype != PointField.FLOAT32 for field in (x_field, y_field, z_field)):
            return None
        return x_field, y_field, z_field

    def cloud_xyz(self, cloud: PointCloud2) -> Optional[np.ndarray]:
        fields = self.point_fields(cloud)
        if fields is None:
            return None
        x_field, y_field, z_field = fields
        endian = ">" if cloud.is_bigendian else "<"
        data_buffer = memoryview(cloud.data)
        return np.stack(
            [
                np.ndarray(
                    shape=(cloud.height, cloud.width),
                    dtype=f"{endian}f4",
                    buffer=data_buffer,
                    offset=field.offset,
                    strides=(cloud.row_step, cloud.point_step),
                ).astype(np.float64, copy=False)
                for field in (x_field, y_field, z_field)
            ],
            axis=2,
        )

    def object_points(
        self,
        image_shape: tuple[int, int],
        object_mask: np.ndarray,
        dilate_mask: bool = True,
    ) -> tuple[Optional[np.ndarray], str]:
        if not self.cloud_fresh():
            return None, "no fresh cloud"
        cloud = self.latest_cloud
        assert cloud is not None
        points = self.cloud_xyz(cloud)
        if points is None:
            return None, "cloud missing float32 xyz"

        mask_source = object_mask.astype(np.uint8)
        if dilate_mask and self.mask_dilate_px > 0:
            kernel_size = self.mask_dilate_px * 2 + 1
            mask_source = cv2.dilate(mask_source, np.ones((kernel_size, kernel_size), dtype=np.uint8))
        mask = cv2.resize(
            mask_source,
            (cloud.width, cloud.height),
            interpolation=cv2.INTER_NEAREST,
        ) > 0
        valid = (
            mask
            & np.isfinite(points).all(axis=2)
            & (points[:, :, 2] >= self.min_depth_m)
            & (points[:, :, 2] <= self.max_depth_m)
        )
        selected = points[valid]
        if selected.shape[0] < self.min_inliers:
            raw_mask_count = int(np.count_nonzero(mask))
            valid_depth_count = int(np.count_nonzero(valid))
            return None, f"too few points mask={raw_mask_count} valid={valid_depth_count} need={self.min_inliers}"
        if selected.shape[0] > self.max_points:
            indices = np.linspace(0, selected.shape[0] - 1, self.max_points).astype(np.int64)
            selected = selected[indices]
        return selected, f"points={selected.shape[0]}"

    def front_plane_from_mask(
        self,
        image_shape: tuple[int, int],
        object_mask: np.ndarray,
    ) -> tuple[Optional[PlaneFit], Optional[float], str]:
        points, status = self.object_points(image_shape, object_mask, dilate_mask=False)
        if points is None:
            return None, None, status

        plane = self.fit_plane_ransac(points)
        if plane is None:
            return None, None, f"{status} plane failed"
        yaw = self.plane_yaw_error(plane)
        return plane, yaw, (
            f"{status} plane_points={points.shape[0]} "
            f"plane_yaw={math.degrees(yaw):+.1f}deg rms={plane.rms_m:.4f}m"
        )

    def object_cluster(self, image_shape: tuple[int, int], object_mask: np.ndarray) -> tuple[Optional[ClusterStats], str]:
        if not self.cloud_fresh():
            return None, "no fresh cloud"
        cloud = self.latest_cloud
        assert cloud is not None
        points_img = self.cloud_xyz(cloud)
        if points_img is None:
            return None, "cloud missing float32 xyz"

        mask_source = object_mask.astype(np.uint8)
        if self.mask_dilate_px > 0:
            kernel_size = self.mask_dilate_px * 2 + 1
            mask_source = cv2.dilate(mask_source, np.ones((kernel_size, kernel_size), dtype=np.uint8))
        mask = cv2.resize(mask_source, (cloud.width, cloud.height), interpolation=cv2.INTER_NEAREST) > 0
        valid = (
            mask
            & np.isfinite(points_img).all(axis=2)
            & (points_img[:, :, 2] >= self.min_depth_m)
            & (points_img[:, :, 2] <= self.max_depth_m)
        )
        if np.count_nonzero(valid) < self.min_inliers:
            return None, f"too few cloud pixels valid={int(np.count_nonzero(valid))} need={self.min_inliers}"

        count, labels, stats, centroids = cv2.connectedComponentsWithStats(valid.astype(np.uint8), connectivity=8)
        if count <= 1:
            return None, "no cloud cluster"

        best_label = None
        best_score = -1.0
        image_center = np.array([cloud.width * 0.5, cloud.height * 0.5], dtype=np.float64)
        for label_idx in range(1, count):
            area = float(stats[label_idx, cv2.CC_STAT_AREA])
            if area < self.min_inliers:
                continue
            center = centroids[label_idx]
            center_distance = float(np.linalg.norm(center - image_center))
            score = area - 0.25 * center_distance
            if score > best_score:
                best_score = score
                best_label = label_idx
        if best_label is None:
            return None, "no large enough cloud cluster"

        cluster_mask = labels == best_label
        points = points_img[cluster_mask]
        if points.shape[0] > self.max_points:
            indices = np.linspace(0, points.shape[0] - 1, self.max_points).astype(np.int64)
            points = points[indices]

        stats_result = self.cluster_stats_from_points(points)
        if stats_result is None:
            return None, "cluster side bands failed"
        return stats_result, (
            f"cluster={stats_result.points.shape[0]} "
            f"left={stats_result.left_count} right={stats_result.right_count}"
        )

    def cluster_stats_from_points(self, points: np.ndarray) -> Optional[ClusterStats]:
        if points.shape[0] < self.min_inliers:
            return None
        centroid = np.median(points, axis=0)
        x_values = points[:, 0]
        y_values = points[:, 1]
        x_min = float(np.percentile(x_values, 5))
        x_max = float(np.percentile(x_values, 95))
        y_min = float(np.percentile(y_values, 5))
        y_max = float(np.percentile(y_values, 95))
        width = max(1e-6, x_max - x_min)
        band_width = max(1e-4, width * float(np.clip(self.side_band_fraction, 0.05, 0.45)))
        left_points = points[x_values <= x_min + band_width]
        right_points = points[x_values >= x_max - band_width]
        if left_points.shape[0] < self.min_side_points or right_points.shape[0] < self.min_side_points:
            return None

        left_mean = np.mean(left_points, axis=0)
        right_mean = np.mean(right_points, axis=0)
        delta = float(left_mean[2] - right_mean[2])
        return ClusterStats(
            points=points,
            centroid=centroid,
            left_mean=left_mean,
            right_mean=right_mean,
            left_count=int(left_points.shape[0]),
            right_count=int(right_points.shape[0]),
            left_right_depth_delta_m=delta,
            width_m=width,
            height_m=max(1e-6, y_max - y_min),
            nearest_depth_m=float(np.min(points[:, 2])),
        )

    def averaged_cluster(self, cluster: ClusterStats) -> ClusterStats:
        self.cluster_history.append(
            (
                cluster.centroid.copy(),
                cluster.left_mean.copy(),
                cluster.right_mean.copy(),
                float(cluster.left_right_depth_delta_m),
                float(cluster.width_m),
                float(cluster.height_m),
                float(cluster.nearest_depth_m),
            )
        )
        centroids = np.array([entry[0] for entry in self.cluster_history], dtype=np.float64)
        left_means = np.array([entry[1] for entry in self.cluster_history], dtype=np.float64)
        right_means = np.array([entry[2] for entry in self.cluster_history], dtype=np.float64)
        return ClusterStats(
            points=cluster.points,
            centroid=np.mean(centroids, axis=0),
            left_mean=np.mean(left_means, axis=0),
            right_mean=np.mean(right_means, axis=0),
            left_count=cluster.left_count,
            right_count=cluster.right_count,
            left_right_depth_delta_m=float(np.mean([entry[3] for entry in self.cluster_history])),
            width_m=float(np.mean([entry[4] for entry in self.cluster_history])),
            height_m=float(np.mean([entry[5] for entry in self.cluster_history])),
            nearest_depth_m=float(np.mean([entry[6] for entry in self.cluster_history])),
        )

    def object_cluster_from_previous(self) -> tuple[Optional[ClusterStats], str]:
        if self.last_cluster_centroid is None:
            return None, "no previous cluster"
        if not self.cloud_fresh():
            return None, "no fresh cloud"
        cloud = self.latest_cloud
        assert cloud is not None
        points_img = self.cloud_xyz(cloud)
        if points_img is None:
            return None, "cloud missing float32 xyz"

        center = self.last_cluster_centroid
        valid = (
            np.isfinite(points_img).all(axis=2)
            & (points_img[:, :, 2] >= self.min_depth_m)
            & (points_img[:, :, 2] <= self.max_depth_m)
            & (np.abs(points_img[:, :, 0] - center[0]) <= self.track_radius_m)
            & (np.abs(points_img[:, :, 1] - center[1]) <= self.track_radius_m)
            & (np.abs(points_img[:, :, 2] - center[2]) <= self.track_depth_radius_m)
        )
        if np.count_nonzero(valid) < self.min_inliers:
            return None, f"track roi too small valid={int(np.count_nonzero(valid))}"

        count, labels, stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8), connectivity=8)
        best_label = None
        best_score = float("inf")
        for label_idx in range(1, count):
            area = int(stats[label_idx, cv2.CC_STAT_AREA])
            if area < self.min_inliers:
                continue
            pts = points_img[labels == label_idx]
            cluster_center = np.median(pts, axis=0)
            score = float(np.linalg.norm(cluster_center - center)) - 0.001 * area
            if score < best_score:
                best_score = score
                best_label = label_idx
        if best_label is None:
            return None, "no tracked cloud cluster"

        points = points_img[labels == best_label]
        if points.shape[0] > self.max_points:
            indices = np.linspace(0, points.shape[0] - 1, self.max_points).astype(np.int64)
            points = points[indices]
        stats_result = self.cluster_stats_from_points(points)
        if stats_result is None:
            return None, f"tracked side bands failed points={points.shape[0]}"
        return stats_result, (
            f"tracked={stats_result.points.shape[0]} "
            f"left={stats_result.left_count} right={stats_result.right_count}"
        )

    def fit_plane_ransac(self, points: np.ndarray) -> Optional[PlaneFit]:
        if points.shape[0] < self.min_inliers:
            return None

        rng = np.random.default_rng(42)
        best_inliers = None
        best_count = 0
        for _ in range(self.ransac_iterations):
            sample_idx = rng.choice(points.shape[0], size=3, replace=False)
            p0, p1, p2 = points[sample_idx]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = float(np.linalg.norm(normal))
            if norm < 1e-6:
                continue
            normal = normal / norm
            distances = np.abs((points - p0) @ normal)
            inliers = distances < self.ransac_threshold_m
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count < self.min_inliers:
            return None

        inlier_points = points[best_inliers]
        centroid = np.median(inlier_points, axis=0)
        centered = inlier_points - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        if normal[2] > 0.0:
            normal = -normal
        distances = centered @ normal
        rms = float(np.sqrt(np.mean(distances * distances)))
        if rms > self.max_plane_rms_m:
            return None

        axes = vh[:2]
        projected = centered @ axes.T
        width = float(np.percentile(projected[:, 0], 95) - np.percentile(projected[:, 0], 5))
        height = float(np.percentile(projected[:, 1], 95) - np.percentile(projected[:, 1], 5))
        return PlaneFit(
            normal=normal,
            centroid=centroid,
            inlier_mask=best_inliers,
            rms_m=rms,
            width_m=max(width, height),
            height_m=min(width, height),
        )

    def plane_yaw_error(self, plane: PlaneFit) -> float:
        return math.atan2(float(plane.normal[0]), max(1e-6, -float(plane.normal[2])))

    def filtered(self, old: Optional[float], new: float) -> float:
        if old is None:
            return new
        alpha = float(np.clip(self.filter_alpha, 0.0, 1.0))
        return (1.0 - alpha) * old + alpha * new

    def command_from_plane(self, plane: PlaneFit) -> tuple[Twist, str, float, float, float]:
        yaw_error = math.atan2(float(plane.normal[0]), max(1e-6, -float(plane.normal[2])))
        lateral_error = -float(plane.centroid[0])
        distance_error = float(plane.centroid[2]) - self.target_distance_m

        self.filtered_yaw = self.filtered(self.filtered_yaw, yaw_error)
        self.filtered_lateral = self.filtered(self.filtered_lateral, lateral_error)
        self.filtered_distance = self.filtered(self.filtered_distance, distance_error)
        yaw = self.filtered_yaw
        lateral = self.filtered_lateral
        distance = self.filtered_distance

        cmd = Twist()
        if abs(yaw) > self.yaw_deadband_rad:
            stage = "yaw"
            cmd.angular.z = float(np.clip(self.yaw_kp * yaw, -self.max_angular_speed, self.max_angular_speed))
            if self.invert_angular:
                cmd.angular.z *= -1.0
        elif abs(lateral) > self.lateral_deadband_m:
            stage = "lateral"
            cmd.linear.y = float(np.clip(self.lateral_kp * lateral, -self.max_lateral_speed, self.max_lateral_speed))
            if self.invert_lateral:
                cmd.linear.y *= -1.0
        elif distance > self.distance_deadband_m:
            stage = "forward"
            cmd.linear.x = float(np.clip(self.forward_kp * distance, 0.0, self.max_forward_speed))
        else:
            stage = "done"
        return cmd, stage, yaw, lateral, distance

    def command_from_cluster(
        self,
        cluster: ClusterStats,
        plane_yaw: Optional[float] = None,
    ) -> tuple[Twist, str, float, float, float]:
        yaw_delta = cluster.left_right_depth_delta_m
        lateral_error = -float(cluster.centroid[0])
        distance_error = float(cluster.nearest_depth_m) - self.target_distance_m
        yaw = float(plane_yaw) if plane_yaw is not None else yaw_delta
        lateral = lateral_error
        distance = distance_error

        cmd = Twist()
        lateral_active = abs(lateral) > self.lateral_deadband_m
        yaw_active = abs(yaw) > (self.yaw_deadband_rad if plane_yaw is not None else self.yaw_depth_deadband_m)
        approach_active = distance > 0.0
        if approach_active:
            cmd.linear.x = float(np.clip(self.approach_speed, 0.0, self.max_forward_speed))
        if lateral_active:
            lateral_command_error = lateral if self.invert_lateral else -lateral
            cmd.linear.y = float(
                np.clip(
                    self.lateral_kp * lateral_command_error,
                    -self.max_lateral_speed,
                    self.max_lateral_speed,
                )
            )
        if yaw_active:
            if plane_yaw is not None:
                # Positive angular.z turns left. The fitted plane yaw uses the same sign.
                cmd.angular.z = float(np.clip(self.yaw_kp * yaw, -self.max_angular_speed, self.max_angular_speed))
            else:
                # Fallback: if the left side is closer, yaw_delta is negative and this commands left.
                cmd.angular.z = float(np.clip(-self.yaw_depth_kp * yaw, -self.max_angular_speed, self.max_angular_speed))
        if approach_active and lateral_active and yaw_active:
            stage = "approach_lateral_plane" if plane_yaw is not None else "approach_lateral_orient"
        elif approach_active and lateral_active:
            stage = "approach_lateral"
        elif approach_active and yaw_active:
            stage = "approach_plane" if plane_yaw is not None else "approach_orient"
        elif approach_active:
            stage = "approach"
        elif lateral_active and yaw_active:
            stage = "lateral_and_plane" if plane_yaw is not None else "lateral_and_orient"
        elif lateral_active:
            stage = "lateral"
        elif yaw_active:
            stage = "orient_plane" if plane_yaw is not None else "orient_pointcloud"
        else:
            stage = "done"
        self.latest_topdown_object_xy = np.array([float(cluster.centroid[2]), -float(cluster.centroid[0])], dtype=np.float64)
        self.latest_topdown_target_xy = np.array([float(cmd.linear.x), float(cmd.linear.y)], dtype=np.float64)
        self.latest_topdown_target_yaw = float(cmd.angular.z)
        return cmd, stage, yaw, lateral, distance

    def yaw_to_quaternion(self, yaw: float) -> tuple[float, float]:
        return math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    def turn_direction(self, yaw: float) -> str:
        if yaw > 0.0:
            return "turn_left"
        if yaw < 0.0:
            return "turn_right"
        return "turn_none"

    def lateral_direction(self, y: float) -> str:
        if y > 0.0:
            return "move_left"
        if y < 0.0:
            return "move_right"
        return "move_none"

    def depth_balance(self, left_right_depth_delta_m: Optional[float]) -> str:
        if left_right_depth_delta_m is None:
            return "depth_unknown"
        if left_right_depth_delta_m < -self.yaw_depth_deadband_m:
            return "left_closer"
        if left_right_depth_delta_m > self.yaw_depth_deadband_m:
            return "right_closer"
        return "depth_equal"

    def motor_step_dt(self, frame_ns: Optional[int]) -> float:
        now_ns = frame_ns or self.get_clock().now().nanoseconds
        if self.last_motor_step_frame_ns is None:
            return min(0.10, self.max_motor_step_dt_sec)
        dt = (now_ns - self.last_motor_step_frame_ns) / 1_000_000_000.0
        return float(np.clip(dt, 0.0, self.max_motor_step_dt_sec))

    def update_base_motor_goal(
        self,
        cluster: ClusterStats,
        cmd: Twist,
        frame_ns: Optional[int],
    ) -> tuple[float, float, float]:
        object_xy = np.array([float(cluster.centroid[2]), -float(cluster.centroid[0])], dtype=np.float64)
        dt = self.motor_step_dt(frame_ns)

        target_xy = np.zeros(2, dtype=np.float64)
        yaw = 0.0
        if abs(cmd.linear.x) > 1e-6:
            target_xy[0] = math.copysign(self.motor_linear_speed * dt, cmd.linear.x)
        if abs(cmd.linear.y) > 1e-6:
            target_xy[1] = math.copysign(self.motor_linear_speed * dt, cmd.linear.y)
        if abs(cmd.angular.z) > 1e-6:
            yaw = math.copysign(self.motor_angular_speed * dt, cmd.angular.z)
        self.latest_topdown_object_xy = object_xy.copy()
        self.latest_topdown_target_xy = target_xy.copy()
        self.latest_topdown_target_yaw = yaw

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.base_frame
        goal_pose.header.stamp = Time(sec=0, nanosec=0)
        goal_pose.pose.position.x = float(target_xy[0])
        goal_pose.pose.position.y = float(target_xy[1])
        goal_pose.pose.position.z = 0.0
        qz, qw = self.yaw_to_quaternion(yaw)
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        self.latest_base_goal = goal_pose

        return float(target_xy[0]), float(target_xy[1]), yaw

    def maybe_send_motor_goal(self, frame_ns: Optional[int] = None) -> None:
        if (
            not self.motion_enabled()
            or not self.use_motor_move
            or self.motor_goal_pending
            or self.motor_active
            or self.await_fresh_image_after_motion
            or self.motor_waiting_after_motion()
        ):
            return
        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_motor_goal_time_ns is not None
            and (now_ns - self.last_motor_goal_time_ns) / 1_000_000_000.0 < self.motor_goal_period_sec
        ):
            return
        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("motor_move action server not available", throttle_duration_sec=1.0)
            return
        if self.latest_base_goal is None:
            return
        if self.base_goal_is_zero(self.latest_base_goal):
            return

        action_goal = MotorMove.Goal()
        action_goal.motor_goal = self.latest_base_goal
        if self.latest_topdown_object_xy is not None and self.latest_topdown_target_xy is not None:
            self.pending_move_start_object_xy = self.latest_topdown_object_xy.copy()
            self.pending_move_target_xy = self.latest_topdown_target_xy.copy()
            self.pending_move_target_yaw = self.latest_topdown_target_yaw
        self.motor_goal_pending = True
        self.motor_active = True
        self.last_motor_goal_time_ns = now_ns
        self.last_motor_step_frame_ns = frame_ns or now_ns
        pose = self.latest_base_goal.pose
        yaw = math.atan2(2.0 * pose.orientation.w * pose.orientation.z, 1.0 - 2.0 * pose.orientation.z * pose.orientation.z)
        self.get_logger().info(
            f"sending motor_move base delta x={pose.position.x:+.3f}m y={pose.position.y:+.3f}m "
            f"{self.lateral_direction(pose.position.y)} yaw={math.degrees(yaw):+.1f}deg {self.turn_direction(yaw)}"
        )
        future = self.action_client.send_goal_async(action_goal)
        future.add_done_callback(self.on_motor_goal_response)

    def on_motor_goal_response(self, future) -> None:
        goal_handle = future.result()
        self.motor_goal_pending = False
        if not goal_handle.accepted:
            self.get_logger().warn("motor_move rejected target goal")
            self.motor_active = False
            self.await_fresh_image_after_motion = self.require_fresh_image_after_motion
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_motor_result)

    def on_motor_result(self, future) -> None:
        result = future.result().result
        self.motor_active = False
        self.await_fresh_image_after_motion = self.require_fresh_image_after_motion
        self.motor_wait_until_ns = (
            self.get_clock().now().nanoseconds + int(self.motor_result_wait_sec * 1_000_000_000)
        )
        self.latest_base_goal = None
        self.filtered_yaw = None
        self.filtered_lateral = None
        self.filtered_distance = None
        self.get_logger().info(
            f"motor_move result success={result.success}; waiting for fresh image and {self.motor_result_wait_sec:.3f}s",
            throttle_duration_sec=1.0,
        )

    def motor_waiting_after_motion(self) -> bool:
        if self.motor_wait_until_ns is None:
            return False
        if self.get_clock().now().nanoseconds < self.motor_wait_until_ns:
            return True
        self.motor_wait_until_ns = None
        return False

    def base_goal_is_zero(self, goal: PoseStamped) -> bool:
        pose = goal.pose
        yaw = math.atan2(2.0 * pose.orientation.w * pose.orientation.z, 1.0 - 2.0 * pose.orientation.z * pose.orientation.z)
        return (
            abs(float(pose.position.x)) < 1e-6
            and abs(float(pose.position.y)) < 1e-6
            and abs(yaw) < 1e-6
        )

    def update_align_progress(
        self,
        centered: bool,
        cluster: Optional[ClusterStats],
        stage: str,
        yaw: Optional[float],
        lateral: Optional[float],
        distance: Optional[float],
    ) -> None:
        if cluster is not None:
            self.align_latest_front_range = float(cluster.nearest_depth_m)
        if not self.align_goal_active():
            return
        if (
            centered
            and not self.motor_active
            and not self.motor_goal_pending
            and not self.await_fresh_image_after_motion
            and not self.motor_waiting_after_motion()
        ):
            self.align_centered_count += 1
        else:
            self.align_centered_count = 0
        self.publish_align_feedback()
        if self.align_centered_count >= self.centered_stable_frames:
            message = (
                f"centered stage={stage} lr_dz={yaw or 0.0:+.4f}m "
                f"lat={lateral or 0.0:+.4f}m nearest_err={distance or 0.0:+.4f}m"
            )
            self.finish_align_goal(True, message)

    def publish_cluster_command(self, cluster: ClusterStats, status: str) -> tuple[Twist, str, float, float, float]:
        cmd, stage, yaw, lateral, distance = self.command_from_cluster(cluster)
        self.last_cluster_centroid = cluster.centroid.copy()
        self.last_cluster_stamp_ns = self.get_clock().now().nanoseconds
        target_x = target_y = target_yaw = None
        if self.use_motor_move:
            frame_ns = self.get_clock().now().nanoseconds
            target_x, target_y, target_yaw = self.update_base_motor_goal(cluster, cmd, frame_ns)
            self.maybe_send_motor_goal(frame_ns)
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        elif self.motion_enabled() and rclpy.ok():
            target_x = float(cmd.linear.x)
            target_y = float(cmd.linear.y)
            target_yaw = float(cmd.angular.z)
            self.set_active_cmd(cmd)
            self.cmd_pub.publish(cmd)
        elif rclpy.ok():
            target_x = float(cmd.linear.x)
            target_y = float(cmd.linear.y)
            target_yaw = float(cmd.angular.z)
            self.set_active_cmd(Twist())
            self.cmd_pub.publish(Twist())
        if self.frame_count % 5 == 0:
            if target_x is None:
                target_text = ""
            elif self.use_motor_move:
                target_text = f" base_delta=({target_x:.3f},{target_y:.3f},{math.degrees(target_yaw):+.1f}deg)"
            else:
                target_text = f" cmd_target=({target_x:.3f},{target_y:.3f}, angular.z={target_yaw:+.2f})"
            direction_text = "" if target_yaw is None else (
                f" {self.lateral_direction(target_y or 0.0)} {self.turn_direction(target_yaw)}"
            )
            self.get_logger().info(
                f"stage={stage} {self.depth_balance(yaw)} lr_dz={yaw} lat={lateral} nearest_err={distance} {status} "
                f"cmd=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.angular.z:+.2f}){target_text}{direction_text}"
            )
        return cmd, stage, yaw, lateral, distance

    def control_from_latest_cloud(self) -> None:
        if not self.controller_active():
            return
        if self.use_motor_move and (
            self.motor_active or self.await_fresh_image_after_motion or self.motor_waiting_after_motion()
        ):
            return
        cluster, status = self.object_cluster_from_previous()
        if cluster is None:
            self.cluster_history.clear()
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.motion_enabled()
                and self.last_cluster_stamp_ns is not None
                and (now_ns - self.last_cluster_stamp_ns) / 1_000_000_000.0 > self.lost_stop_after_sec
            ):
                self.stop()
            self.get_logger().warn(f"cloud tracking lost: {status}", throttle_duration_sec=1.0)
            return

        averaged = self.averaged_cluster(cluster)
        cmd, stage, yaw, lateral, distance = self.publish_cluster_command(averaged, status)
        centered = stage == "done"
        self.update_align_progress(centered, cluster, stage, yaw, lateral, distance)
        if self.latest_image is not None and self.latest_object_mask is not None:
            debug = self.draw_debug(
                self.latest_image,
                self.latest_detection,
                self.latest_object_mask,
                None,
                stage,
                cmd,
                yaw,
                lateral,
                distance,
                cluster,
                None,
            )
            cv2.imwrite(self.output_path, debug)

    def draw_debug(
        self,
        image: np.ndarray,
        detection: Optional[tuple],
        object_mask: np.ndarray,
        plane: Optional[PlaneFit],
        stage: str,
        cmd: Twist,
        yaw: Optional[float],
        lateral: Optional[float],
        distance: Optional[float],
        cluster: Optional[ClusterStats] = None,
        depth_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        debug = image.copy()
        if depth_mask is not None:
            depth_overlay = debug.copy()
            depth_overlay[depth_mask] = (255, 255, 0)
            debug = cv2.addWeighted(depth_overlay, 0.16, debug, 0.84, 0.0)
        overlay = debug.copy()
        overlay[object_mask] = (0, 255, 0)
        debug = cv2.addWeighted(overlay, 0.25, debug, 0.75, 0.0)

        if detection is not None:
            _, box, cx, cy, *_ = detection
            cv2.drawContours(debug, [box], 0, (0, 0, 255), 4)
            cv2.drawMarker(debug, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 26, 3)

        if cluster is None:
            label = f"{stage}: no cluster"
        else:
            if plane is None:
                yaw_text = f"lr_dz={yaw or 0.0:+.3f}m"
                plane_text = "plane=none"
            else:
                plane_yaw = self.plane_yaw_error(plane)
                yaw_text = f"plane_yaw={math.degrees(plane_yaw):+.1f}deg"
                plane_text = f"plane rms={plane.rms_m:.4f}m inliers={int(np.count_nonzero(plane.inlier_mask))}"
            label = (
                f"{stage} {yaw_text} "
                f"Lz/Rz={cluster.left_mean[2]:.3f}/{cluster.right_mean[2]:.3f} "
                f"lat={lateral or 0.0:+.3f}m nearest={cluster.nearest_depth_m:.3f}m err={distance or 0.0:+.3f}m "
                f"pts={cluster.points.shape[0]} {plane_text} "
                f"cmd x={cmd.linear.x:+.2f} y={cmd.linear.y:+.2f} z={cmd.angular.z:+.2f}"
            )

        cv2.putText(debug, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(debug, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
        return debug

    def draw_topdown_debug(
        self,
        cluster: Optional[ClusterStats],
        target_x: Optional[float],
        target_y: Optional[float],
        target_yaw: Optional[float],
        stage: str,
        plane: Optional[PlaneFit] = None,
    ) -> np.ndarray:
        size = 640
        scale = 650.0
        origin = np.array([size // 2, size - 90], dtype=np.float64)
        image = np.full((size, size, 3), 245, dtype=np.uint8)

        def to_px(x_forward: float, y_left: float) -> tuple[int, int]:
            px = origin + np.array([-y_left * scale, -x_forward * scale], dtype=np.float64)
            return int(np.clip(px[0], 0, size - 1)), int(np.clip(px[1], 0, size - 1))

        for meters in (0.1, 0.2, 0.3, 0.4, 0.5):
            y = to_px(meters, 0.0)[1]
            cv2.line(image, (20, y), (size - 20, y), (225, 225, 225), 1)
            cv2.putText(image, f"{meters:.1f}m", (24, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
        for lateral in (-0.3, -0.2, -0.1, 0.1, 0.2, 0.3):
            x = to_px(0.0, lateral)[0]
            cv2.line(image, (x, 35), (x, size - 35), (230, 230, 230), 1)

        cv2.line(image, to_px(0.0, 0.0), to_px(0.55, 0.0), (180, 180, 180), 2)
        cv2.line(image, to_px(self.target_distance_m, -0.35), to_px(self.target_distance_m, 0.35), (80, 180, 80), 2)
        cv2.circle(image, to_px(0.0, 0.0), 12, (30, 30, 30), -1)
        cv2.putText(image, "robot", (int(origin[0]) + 16, int(origin[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 2)

        object_xy = self.latest_topdown_object_xy
        if object_xy is None and cluster is not None:
            object_xy = np.array([float(cluster.centroid[2]), -float(cluster.centroid[0])], dtype=np.float64)
        if object_xy is not None:
            object_px = to_px(float(object_xy[0]), float(object_xy[1]))
            cv2.circle(image, object_px, 14, (0, 120, 255), -1)
            cv2.circle(image, object_px, 20, (0, 120, 255), 2)
            cv2.putText(
                image,
                f"box x={object_xy[0]:+.3f} y={object_xy[1]:+.3f}",
                (object_px[0] + 14, max(25, object_px[1] - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 80, 180),
                2,
            )

        if cluster is not None:
            left_xy = np.array([float(cluster.left_mean[2]), -float(cluster.left_mean[0])], dtype=np.float64)
            right_xy = np.array([float(cluster.right_mean[2]), -float(cluster.right_mean[0])], dtype=np.float64)
            left_px = to_px(float(left_xy[0]), float(left_xy[1]))
            right_px = to_px(float(right_xy[0]), float(right_xy[1]))
            cv2.line(image, left_px, right_px, (40, 40, 220), 3)
            cv2.circle(image, left_px, 8, (255, 0, 255), -1)
            cv2.circle(image, right_px, 8, (255, 180, 0), -1)
            cv2.putText(image, "L", (left_px[0] + 8, left_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 0, 180), 2)
            cv2.putText(image, "R", (right_px[0] + 8, right_px[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 100, 0), 2)
            orientation_label = (
                f"box orientation: {self.depth_balance(cluster.left_right_depth_delta_m)} "
                f"lr_dz={cluster.left_right_depth_delta_m:+.3f}m"
            )
            cv2.putText(image, orientation_label, (20, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 160), 2)

        if plane is not None:
            center_xy = np.array([float(plane.centroid[2]), -float(plane.centroid[0])], dtype=np.float64)
            normal_xy = np.array([float(plane.normal[2]), -float(plane.normal[0])], dtype=np.float64)
            norm = float(np.linalg.norm(normal_xy))
            if norm > 1e-6:
                normal_xy /= norm
                tangent_xy = np.array([-normal_xy[1], normal_xy[0]], dtype=np.float64)
                half_width = max(0.04, min(0.25, plane.width_m * 0.5))
                a_xy = center_xy - tangent_xy * half_width
                b_xy = center_xy + tangent_xy * half_width
                center_px = to_px(float(center_xy[0]), float(center_xy[1]))
                cv2.line(
                    image,
                    to_px(float(a_xy[0]), float(a_xy[1])),
                    to_px(float(b_xy[0]), float(b_xy[1])),
                    (80, 0, 180),
                    4,
                )
                cv2.arrowedLine(
                    image,
                    center_px,
                    to_px(float(center_xy[0] - normal_xy[0] * 0.08), float(center_xy[1] - normal_xy[1] * 0.08)),
                    (180, 40, 180),
                    2,
                    tipLength=0.35,
                )
                plane_yaw = self.plane_yaw_error(plane)
                cv2.putText(
                    image,
                    f"ransac plane yaw={math.degrees(plane_yaw):+.1f}deg rms={plane.rms_m:.3f}m",
                    (20, 112),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (100, 0, 150),
                    2,
                )

        if self.last_move_start_object_xy is not None:
            start_px = to_px(float(self.last_move_start_object_xy[0]), float(self.last_move_start_object_xy[1]))
            cv2.circle(image, start_px, 9, (255, 80, 0), -1)
            cv2.putText(image, "before", (start_px[0] + 10, start_px[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 70, 0), 2)
        if self.last_move_end_object_xy is not None:
            end_px = to_px(float(self.last_move_end_object_xy[0]), float(self.last_move_end_object_xy[1]))
            cv2.circle(image, end_px, 9, (0, 160, 0), -1)
            cv2.putText(image, "after", (end_px[0] + 10, end_px[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 120, 0), 2)
            if self.last_move_start_object_xy is not None:
                cv2.arrowedLine(image, start_px, end_px, (0, 140, 0), 2, tipLength=0.2)

        if target_x is not None and target_y is not None and target_yaw is not None:
            target_px = to_px(float(target_x), float(target_y))
            cv2.arrowedLine(image, to_px(0.0, 0.0), target_px, (220, 50, 50), 3, tipLength=0.25)
            yaw_radius = 54
            yaw_text = f"angular.z={target_yaw:+.2f} {self.turn_direction(target_yaw)}"
            cv2.ellipse(image, to_px(0.0, 0.0), (yaw_radius, yaw_radius), 0, 0, float(np.clip(target_yaw * 180.0, -90.0, 90.0)), (150, 40, 200), 2)
            cv2.putText(
                image,
                f"cmd_vel x={target_x:+.3f} y={target_y:+.3f} {self.lateral_direction(target_y)}",
                (20, size - 44),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (170, 40, 40),
                2,
            )
            cv2.putText(image, yaw_text, (20, size - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (130, 40, 160), 2)
        elif self.last_move_target_xy is not None:
            target_px = to_px(float(self.last_move_target_xy[0]), float(self.last_move_target_xy[1]))
            cv2.arrowedLine(image, to_px(0.0, 0.0), target_px, (220, 50, 50), 2, tipLength=0.25)
            cv2.putText(
                image,
                f"last cmd x={self.last_move_target_xy[0]:+.3f} y={self.last_move_target_xy[1]:+.3f} "
                f"{self.lateral_direction(float(self.last_move_target_xy[1]))}",
                (20, size - 44),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (170, 40, 40),
                2,
            )
            cv2.putText(
                image,
                f"angular.z={self.last_move_target_yaw:+.2f} {self.turn_direction(self.last_move_target_yaw)}",
                (20, size - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (130, 40, 160),
                2,
            )

        cv2.putText(image, f"stage={stage}", (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (20, 20, 20), 2)
        cv2.putText(image, "top-down: x forward, screen right is +y (camera point cloud only)", (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (70, 70, 70), 1)
        return image

    def on_image(self, msg: Image) -> None:
        if not self.controller_active():
            return
        if self.too_old(msg, self.max_image_age_sec, "image"):
            if self.motion_enabled():
                self.stop()
            return
        image_stamp_ns = self.stamp_to_ns(msg)
        if image_stamp_ns is not None:
            if (
                self.last_processed_image_stamp_ns is not None
                and image_stamp_ns <= self.last_processed_image_stamp_ns
            ):
                self.get_logger().warn("dropping out-of-order image", throttle_duration_sec=1.0)
                if self.motion_enabled():
                    self.stop()
                return
            self.last_processed_image_stamp_ns = image_stamp_ns

        image = self.image_from_msg(msg)
        if image is None:
            return
        self.latest_image = image

        detection, mask, depth_mask = self.detect_gray_object(image)
        plane = None
        cluster = None
        cmd = Twist()
        stage = "detect"
        yaw = lateral = distance = None
        target_x = target_y = target_yaw = None
        status = ""
        was_awaiting_after_motion = self.await_fresh_image_after_motion

        if detection is not None:
            contour, *_ = detection
            object_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], 0, 255, cv2.FILLED)
            object_mask = object_mask > 0
            self.latest_detection = detection
            self.latest_object_mask = object_mask
            cluster, status = self.object_cluster(image.shape[:2], object_mask)
            if cluster is not None:
                plane, plane_yaw, plane_status = self.front_plane_from_mask(image.shape[:2], object_mask)
                status = f"{status} {plane_status}"
                self.last_cluster_centroid = cluster.centroid.copy()
                self.last_cluster_stamp_ns = self.get_clock().now().nanoseconds
                averaged = self.averaged_cluster(cluster)
                if was_awaiting_after_motion and self.pending_move_target_xy is not None:
                    self.last_move_start_object_xy = (
                        None if self.pending_move_start_object_xy is None else self.pending_move_start_object_xy.copy()
                    )
                    self.last_move_target_xy = self.pending_move_target_xy.copy()
                    self.last_move_target_yaw = self.pending_move_target_yaw
                    self.last_move_end_object_xy = np.array(
                        [float(averaged.centroid[2]), -float(averaged.centroid[0])],
                        dtype=np.float64,
                    )
                cmd, stage, yaw, lateral, distance = self.command_from_cluster(averaged, plane_yaw)
                if self.use_motor_move:
                    target_x, target_y, target_yaw = self.update_base_motor_goal(averaged, cmd, image_stamp_ns)
                    self.await_fresh_image_after_motion = False
                    self.maybe_send_motor_goal(image_stamp_ns)
                else:
                    previous_object_xy = None if self.latest_topdown_object_xy is None else self.latest_topdown_object_xy.copy()
                    target_x = float(cmd.linear.x)
                    target_y = float(cmd.linear.y)
                    target_yaw = float(cmd.angular.z)
                    if previous_object_xy is not None:
                        self.last_move_start_object_xy = previous_object_xy
                        self.last_move_target_xy = np.array([target_x, target_y], dtype=np.float64)
                        self.last_move_target_yaw = target_yaw
                        self.last_move_end_object_xy = self.latest_topdown_object_xy.copy()
                if self.cloud_tracking_enabled and self.use_motor_move:
                    yaw = averaged.left_right_depth_delta_m
                    lateral = -float(averaged.centroid[0])
                    distance = float(averaged.nearest_depth_m) - self.target_distance_m
                    centered = (
                        target_x is not None
                        and abs(target_x) < 1e-6
                        and abs(target_y or 0.0) < 1e-6
                        and abs(target_yaw or 0.0) < 1e-6
                    )
                    stage = "done" if centered else "seed"
                cluster = averaged
            else:
                self.cluster_history.clear()
                stage = "cluster"
        else:
            self.cluster_history.clear()
            object_mask = mask
            self.latest_detection = None
            self.latest_object_mask = object_mask
            self.filtered_yaw = None
            self.filtered_lateral = None
            self.filtered_distance = None

        if self.use_motor_move:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        elif self.motion_enabled() and rclpy.ok():
            self.set_active_cmd(cmd)
            self.cmd_pub.publish(cmd)
        elif rclpy.ok():
            self.set_active_cmd(Twist())
            self.cmd_pub.publish(Twist())

        debug = self.draw_debug(image, detection, object_mask, plane, stage, cmd, yaw, lateral, distance, cluster, depth_mask)
        topdown = self.draw_topdown_debug(cluster, target_x, target_y, target_yaw, stage, plane)
        self.update_align_progress(stage == "done", cluster, stage, yaw, lateral, distance)
        self.frame_count += 1
        cv2.imwrite(self.output_path, debug)
        cv2.imwrite(self.topdown_output_path, topdown)
        if self.frame_count % 5 == 0:
            if target_x is None:
                target_text = ""
            elif self.use_motor_move:
                target_text = f" base_delta=({target_x:.3f},{target_y:.3f},{math.degrees(target_yaw):+.1f}deg)"
            else:
                target_text = f" cmd_target=({target_x:.3f},{target_y:.3f}, angular.z={target_yaw:+.2f})"
            direction_text = "" if target_yaw is None else (
                f" {self.lateral_direction(target_y or 0.0)} {self.turn_direction(target_yaw)}"
            )
            yaw_text = (
                f"plane_yaw={math.degrees(yaw):+.1f}deg"
                if plane is not None and yaw is not None
                else f"lr_dz={yaw}"
            )
            self.get_logger().info(
                f"stage={stage} {yaw_text} lat={lateral} nearest_err={distance} {status} "
                f"cmd=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.angular.z:+.2f}){target_text}{direction_text}"
            )

        if self.show_gui:
            cv2.imshow(self.window_name, debug)
            cv2.imshow(self.topdown_window_name, topdown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = GrayBoxPlaneController()
    try:
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.close()
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
