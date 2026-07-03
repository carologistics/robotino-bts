#!/usr/bin/env python3
"""Servo to a gray box using a RANSAC plane fitted to the masked point cloud."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from motor_move_msgs.action import MotorMove, MoveToShelf
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from tf2_ros import TransformBroadcaster


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
        self.declare_parameter("show_gui", True)
        self.declare_parameter("window_name", "gray box plane controller")

        self.declare_parameter("enable_motion", False)
        self.declare_parameter("max_image_age_sec", 1.00)
        self.declare_parameter("max_cloud_age_sec", 1.20)
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

        self.declare_parameter("target_distance_m", 0.22)
        self.declare_parameter("yaw_deadband_deg", 3.0)
        self.declare_parameter("lateral_deadband_m", 0.005)
        self.declare_parameter("distance_deadband_m", 0.025)
        self.declare_parameter("max_angular_speed", 0.10)
        self.declare_parameter("max_lateral_speed", 0.15)
        self.declare_parameter("max_forward_speed", 0.10)
        self.declare_parameter("side_band_fraction", 0.20)
        self.declare_parameter("min_side_points", 8)
        self.declare_parameter("cloud_tracking_enabled", True)
        self.declare_parameter("use_motor_move", True)
        self.declare_parameter("motor_goal_period_sec", 1.0)
        self.declare_parameter("require_fresh_image_after_motion", True)
        self.declare_parameter("track_radius_m", 0.18)
        self.declare_parameter("track_depth_radius_m", 0.25)
        self.declare_parameter("lost_stop_after_sec", 0.70)
        self.declare_parameter("yaw_depth_deadband_m", 0.010)
        self.declare_parameter("yaw_depth_kp", 2.0)
        self.declare_parameter("yaw_kp", 0.9)
        self.declare_parameter("lateral_kp", 0.8)
        self.declare_parameter("forward_kp", 0.45)
        self.declare_parameter("invert_angular", True)
        self.declare_parameter("invert_lateral", True)
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
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.window_name = str(self.get_parameter("window_name").value)

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
        self.yaw_deadband_rad = math.radians(float(self.get_parameter("yaw_deadband_deg").value))
        self.lateral_deadband_m = float(self.get_parameter("lateral_deadband_m").value)
        self.distance_deadband_m = float(self.get_parameter("distance_deadband_m").value)
        self.max_angular_speed = abs(float(self.get_parameter("max_angular_speed").value))
        self.max_lateral_speed = abs(float(self.get_parameter("max_lateral_speed").value))
        self.max_forward_speed = abs(float(self.get_parameter("max_forward_speed").value))
        self.side_band_fraction = float(self.get_parameter("side_band_fraction").value)
        self.min_side_points = int(self.get_parameter("min_side_points").value)
        self.cloud_tracking_enabled = bool(self.get_parameter("cloud_tracking_enabled").value)
        self.use_motor_move = bool(self.get_parameter("use_motor_move").value)
        self.motor_goal_period_sec = float(self.get_parameter("motor_goal_period_sec").value)
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
        self.motor_goal_pending = False
        self.motor_active = False
        self.await_fresh_image_after_motion = False
        self.latest_base_goal: Optional[PoseStamped] = None
        self.align_goal_handle = None
        self.align_started_ns: Optional[int] = None
        self.align_centered_count = 0
        self.align_latest_front_range = float("nan")
        self.frame_count = 0

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.action_client = ActionClient(self, MotorMove, self.motor_action)
        self.align_action_server = ActionServer(
            self,
            MoveToShelf,
            self.align_action,
            goal_callback=self.on_align_goal,
            cancel_callback=self.on_align_cancel,
            handle_accepted_callback=self.on_align_accepted,
        )
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_align_timer)

        if self.show_gui:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 960, 540)

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

    def stop(self) -> None:
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

    def object_points(self, image_shape: tuple[int, int], object_mask: np.ndarray) -> tuple[Optional[np.ndarray], str]:
        if not self.cloud_fresh():
            return None, "no fresh cloud"
        cloud = self.latest_cloud
        assert cloud is not None
        points = self.cloud_xyz(cloud)
        if points is None:
            return None, "cloud missing float32 xyz"

        mask_source = object_mask.astype(np.uint8)
        if self.mask_dilate_px > 0:
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

    def command_from_cluster(self, cluster: ClusterStats) -> tuple[Twist, str, float, float, float]:
        yaw_delta = cluster.left_right_depth_delta_m
        lateral_error = -float(cluster.centroid[0])
        distance_error = float(cluster.centroid[2]) - self.target_distance_m

        self.filtered_yaw = self.filtered(self.filtered_yaw, yaw_delta)
        self.filtered_lateral = self.filtered(self.filtered_lateral, lateral_error)
        self.filtered_distance = self.filtered(self.filtered_distance, distance_error)
        yaw = self.filtered_yaw
        lateral = self.filtered_lateral
        distance = self.filtered_distance

        cmd = Twist()
        if abs(yaw) > self.yaw_depth_deadband_m:
            stage = "yaw_lr_depth"
            cmd.angular.z = float(np.clip(self.yaw_depth_kp * yaw, -self.max_angular_speed, self.max_angular_speed))
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

    def yaw_to_quaternion(self, yaw: float) -> tuple[float, float]:
        return math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    def update_base_motor_goal(self, cluster: ClusterStats) -> tuple[float, float, float]:
        object_xy = np.array([float(cluster.centroid[2]), -float(cluster.centroid[0])], dtype=np.float64)
        forward_error = float(object_xy[0] - self.target_distance_m)
        lateral_error = float(object_xy[1])

        if abs(cluster.left_right_depth_delta_m) < self.yaw_depth_deadband_m:
            yaw_error = 0.0
        else:
            yaw_error = -math.atan2(float(cluster.left_right_depth_delta_m), max(float(cluster.width_m), 1e-3))

        target_xy = np.zeros(2, dtype=np.float64)
        yaw = 0.0
        if abs(lateral_error) >= self.lateral_deadband_m:
            target_xy[1] = lateral_error
        elif abs(yaw_error) >= self.yaw_deadband_rad:
            yaw = yaw_error
        elif abs(forward_error) >= self.distance_deadband_m:
            target_xy[0] = forward_error

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

        # Keep publishing a TF marker for visualization/debug; motor_move receives
        # the relative base_link pose directly.
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = float(target_xy[0])
        transform.transform.translation.y = float(target_xy[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        if rclpy.ok():
            self.tf_broadcaster.sendTransform(transform)
        return float(target_xy[0]), float(target_xy[1]), yaw

    def maybe_send_motor_goal(self) -> None:
        if (
            not self.motion_enabled()
            or not self.use_motor_move
            or self.motor_goal_pending
            or self.motor_active
            or self.await_fresh_image_after_motion
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
        self.motor_goal_pending = True
        self.motor_active = True
        self.last_motor_goal_time_ns = now_ns
        pose = self.latest_base_goal.pose
        yaw = math.atan2(2.0 * pose.orientation.w * pose.orientation.z, 1.0 - 2.0 * pose.orientation.z * pose.orientation.z)
        self.get_logger().info(
            f"sending motor_move base delta x={pose.position.x:+.3f}m y={pose.position.y:+.3f}m yaw={math.degrees(yaw):+.1f}deg"
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
        self.latest_base_goal = None
        self.filtered_yaw = None
        self.filtered_lateral = None
        self.filtered_distance = None
        self.get_logger().info(f"motor_move result success={result.success}; waiting for fresh image", throttle_duration_sec=1.0)

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
            self.align_latest_front_range = float(cluster.centroid[2])
        if not self.align_goal_active():
            return
        if centered and not self.motor_active and not self.motor_goal_pending and not self.await_fresh_image_after_motion:
            self.align_centered_count += 1
        else:
            self.align_centered_count = 0
        self.publish_align_feedback()
        if self.align_centered_count >= self.centered_stable_frames:
            message = (
                f"centered stage={stage} lr_dz={yaw or 0.0:+.4f}m "
                f"lat={lateral or 0.0:+.4f}m front_err={distance or 0.0:+.4f}m"
            )
            self.finish_align_goal(True, message)

    def publish_cluster_command(self, cluster: ClusterStats, status: str) -> tuple[Twist, str, float, float, float]:
        cmd, stage, yaw, lateral, distance = self.command_from_cluster(cluster)
        self.last_cluster_centroid = cluster.centroid.copy()
        self.last_cluster_stamp_ns = self.get_clock().now().nanoseconds
        target_x = target_y = target_yaw = None
        if self.use_motor_move:
            target_x, target_y, target_yaw = self.update_base_motor_goal(cluster)
            self.maybe_send_motor_goal()
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        elif self.motion_enabled() and rclpy.ok():
            self.cmd_pub.publish(cmd)
        elif rclpy.ok():
            self.cmd_pub.publish(Twist())
        if self.frame_count % 5 == 0:
            target_text = "" if target_x is None else f" base_delta=({target_x:.3f},{target_y:.3f},{math.degrees(target_yaw):+.1f}deg)"
            self.get_logger().info(
                f"stage={stage} lr_dz={yaw} lat={lateral} front_err={distance} {status} "
                f"cmd=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.angular.z:+.2f}){target_text}"
            )
        return cmd, stage, yaw, lateral, distance

    def control_from_latest_cloud(self) -> None:
        if not self.controller_active():
            return
        if self.use_motor_move and (self.motor_active or self.await_fresh_image_after_motion):
            return
        cluster, status = self.object_cluster_from_previous()
        if cluster is None:
            now_ns = self.get_clock().now().nanoseconds
            if (
                self.motion_enabled()
                and self.last_cluster_stamp_ns is not None
                and (now_ns - self.last_cluster_stamp_ns) / 1_000_000_000.0 > self.lost_stop_after_sec
            ):
                self.stop()
            self.get_logger().warn(f"cloud tracking lost: {status}", throttle_duration_sec=1.0)
            return

        cmd, stage, yaw, lateral, distance = self.publish_cluster_command(cluster, status)
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
            label = (
                f"{stage} lr_dz={yaw or 0.0:+.3f}m "
                f"Lz/Rz={cluster.left_mean[2]:.3f}/{cluster.right_mean[2]:.3f} "
                f"lat={lateral or 0.0:+.3f}m front={cluster.centroid[2]:.3f}m err={distance or 0.0:+.3f}m "
                f"pts={cluster.points.shape[0]} range<={self.max_depth_m:.1f}m "
                f"cmd x={cmd.linear.x:+.2f} y={cmd.linear.y:+.2f} z={cmd.angular.z:+.2f}"
            )

        cv2.putText(debug, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(debug, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)
        return debug

    def on_image(self, msg: Image) -> None:
        if not self.controller_active():
            return
        if self.too_old(msg, self.max_image_age_sec, "image"):
            if self.motion_enabled():
                self.stop()
            return

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

        if detection is not None:
            contour, *_ = detection
            object_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(object_mask, [contour], 0, 255, cv2.FILLED)
            object_mask = object_mask > 0
            self.latest_detection = detection
            self.latest_object_mask = object_mask
            cluster, status = self.object_cluster(image.shape[:2], object_mask)
            if cluster is not None:
                self.last_cluster_centroid = cluster.centroid.copy()
                self.last_cluster_stamp_ns = self.get_clock().now().nanoseconds
                if self.use_motor_move:
                    target_x, target_y, target_yaw = self.update_base_motor_goal(cluster)
                    self.await_fresh_image_after_motion = False
                    self.maybe_send_motor_goal()
                if self.cloud_tracking_enabled:
                    yaw = cluster.left_right_depth_delta_m
                    lateral = -float(cluster.centroid[0])
                    distance = float(cluster.centroid[2]) - self.target_distance_m
                    centered = (
                        target_x is not None
                        and abs(target_x) < 1e-6
                        and abs(target_y or 0.0) < 1e-6
                        and abs(target_yaw or 0.0) < 1e-6
                    )
                    stage = "done" if centered else "seed"
                else:
                    cmd, stage, yaw, lateral, distance = self.command_from_cluster(cluster)
            else:
                stage = "cluster"
        else:
            object_mask = mask
            self.latest_detection = None
            self.latest_object_mask = object_mask
            self.filtered_yaw = None
            self.filtered_lateral = None
            self.filtered_distance = None

        if self.use_motor_move:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        elif self.cloud_tracking_enabled:
            if rclpy.ok() and not self.motion_enabled():
                self.cmd_pub.publish(Twist())
        elif self.motion_enabled() and rclpy.ok():
            self.cmd_pub.publish(cmd)
        elif rclpy.ok():
            self.cmd_pub.publish(Twist())

        debug = self.draw_debug(image, detection, object_mask, plane, stage, cmd, yaw, lateral, distance, cluster, depth_mask)
        self.update_align_progress(stage == "done", cluster, stage, yaw, lateral, distance)
        self.frame_count += 1
        cv2.imwrite(self.output_path, debug)
        if self.frame_count % 5 == 0:
            target_text = "" if target_x is None else f" base_delta=({target_x:.3f},{target_y:.3f},{math.degrees(target_yaw):+.1f}deg)"
            self.get_logger().info(
                f"stage={stage} lr_dz={yaw} lat={lateral} front_err={distance} {status} "
                f"cmd=({cmd.linear.x:+.2f},{cmd.linear.y:+.2f},{cmd.angular.z:+.2f}){target_text}"
            )

        if self.show_gui:
            cv2.imshow(self.window_name, debug)
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
