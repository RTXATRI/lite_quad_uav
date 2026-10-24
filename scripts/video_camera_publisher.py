#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _bool_param(name, default):
    value = _param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ["1", "true", "yes", "on"]
    return bool(value)


def _make_camera_info(width, height, frame_id, fx, fy, cx, cy):
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.K = [
        float(fx), 0.0, float(cx),
        0.0, float(fy), float(cy),
        0.0, 0.0, 1.0,
    ]
    msg.R = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    msg.P = [
        float(fx), 0.0, float(cx), 0.0,
        0.0, float(fy), float(cy), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return msg


def _derive_output_path(source_video_path):
    source_dir = os.path.dirname(source_video_path)
    source_name = os.path.basename(source_video_path)
    stem, ext = os.path.splitext(source_name)
    if not ext:
        ext = ".mp4"
    return os.path.join(source_dir, stem + "_detected" + ext)


def _derive_trajectory_path(source_video_path):
    source_dir = os.path.dirname(source_video_path)
    source_name = os.path.basename(source_video_path)
    stem, _ = os.path.splitext(source_name)
    return os.path.join(source_dir, stem + "_camera_trajectory.csv")


def _select_fourcc(output_path, configured_fourcc):
    configured_fourcc = str(configured_fourcc).strip()
    if configured_fourcc and configured_fourcc.lower() != "auto":
        return configured_fourcc[:4]

    ext = os.path.splitext(output_path)[1].lower()
    if ext in [".mp4", ".m4v", ".mov"]:
        return "mp4v"
    if ext in [".avi", ".mkv"]:
        return "XVID"
    return "mp4v"


def _time_close(a, b, tolerance_sec):
    if a.is_zero() or b.is_zero():
        return False
    try:
        return abs((a - b).to_sec()) <= tolerance_sec
    except Exception:
        return False


def _normalize_quaternion(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return None
    return q.x / norm, q.y / norm, q.z / norm, q.w / norm


def _quaternion_to_matrix(q):
    normalized = _normalize_quaternion(q)
    if normalized is None:
        return None

    x, y, z, w = normalized
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _transform_point(rotation, translation, point):
    return (
        rotation[0][0] * point[0] + rotation[0][1] * point[1] + rotation[0][2] * point[2] + translation[0],
        rotation[1][0] * point[0] + rotation[1][1] * point[1] + rotation[1][2] * point[2] + translation[1],
        rotation[2][0] * point[0] + rotation[2][1] * point[1] + rotation[2][2] * point[2] + translation[2],
    )


def _valid_pixel(pixel):
    return (
        pixel is not None
        and math.isfinite(pixel[0])
        and math.isfinite(pixel[1])
    )


def _parse_vector3_param(name, default):
    value = _param(name, default)
    if isinstance(value, str):
        text = value.replace(",", " ")
        parts = [p for p in text.split() if p]
    elif isinstance(value, (list, tuple)):
        parts = value
    else:
        parts = default

    try:
        parsed = [float(parts[0]), float(parts[1]), float(parts[2])]
    except Exception:
        rospy.logwarn("[VideoCameraPublisher] invalid %s=%s, using default %s", name, str(value), str(default))
        parsed = [float(default[0]), float(default[1]), float(default[2])]
    return np.array(parsed, dtype=float)


def _rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def _matrix_to_rpy(rotation):
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1.0e-6
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _matrix_to_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([x, y, z, w], dtype=float))


def _quat_normalize(quat):
    if quat is None:
        return None
    norm = float(np.linalg.norm(quat))
    if not math.isfinite(norm) or norm <= 1.0e-9:
        return None
    return quat / norm


def _quat_angle(a, b):
    a = _quat_normalize(a)
    b = _quat_normalize(b)
    if a is None or b is None:
        return math.pi
    dot = abs(float(np.dot(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _weighted_average_quaternions(quaternions, weights):
    if not quaternions:
        return None
    reference = _quat_normalize(quaternions[0])
    if reference is None:
        return None

    accum = np.zeros(4, dtype=float)
    for quat, weight in zip(quaternions, weights):
        normalized = _quat_normalize(quat)
        if normalized is None:
            continue
        if float(np.dot(reference, normalized)) < 0.0:
            normalized = -normalized
        accum += float(weight) * normalized
    return _quat_normalize(accum)


def _quat_blend(current, target, alpha):
    current = _quat_normalize(current)
    target = _quat_normalize(target)
    if current is None:
        return target
    if target is None:
        return current
    if float(np.dot(current, target)) < 0.0:
        target = -target
    blended = (1.0 - alpha) * current + alpha * target
    return _quat_normalize(blended)


def _make_transform(rotation, translation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def _invert_transform(transform):
    inverse = np.eye(4, dtype=float)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T.dot(translation)
    return inverse


def _pose_to_transform(pose):
    rotation_list = _quaternion_to_matrix(pose.orientation)
    if rotation_list is None:
        return None
    rotation = np.array(rotation_list, dtype=float)
    translation = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    if not np.all(np.isfinite(translation)):
        return None
    return _make_transform(rotation, translation)


def _safe_float(value):
    return float(value) if value is not None and math.isfinite(float(value)) else float("nan")


def _clamp(value, low, high):
    return max(low, min(high, value))


def _wrap_to_pi(angle):
    if not math.isfinite(angle):
        return 0.0
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def _yaw_to_quat(yaw):
    half = 0.5 * _wrap_to_pi(yaw)
    return np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=float)


def _finite_array(values):
    return np.all(np.isfinite(values))


def _safe_inverse(matrix):
    try:
        return np.linalg.inv(matrix)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(matrix)


def _polygon_area(points):
    if len(points) < 3:
        return 0.0
    area = 0.0
    for idx, point in enumerate(points):
        nxt = points[(idx + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return abs(area) * 0.5


def _format_id_list(values):
    return ";".join(str(x) for x in values) if values else ""


class CameraTrajectoryResult:
    def __init__(self):
        self.state = "NO_TARGET"
        self.valid = False
        self.confidence = 0.0
        self.point_source = "LOST"
        self.status = "LOST"
        self.used_tag_ids = []
        self.rejected_tag_ids = []
        self.camera_position = np.array([float("nan"), float("nan"), float("nan")], dtype=float)
        self.camera_rpy = np.array([float("nan"), float("nan"), float("nan")], dtype=float)
        self.camera_quat = None
        self.raw_camera_position = np.array([float("nan"), float("nan"), float("nan")], dtype=float)
        self.residual_position = float("nan")
        self.detection_count = 0
        self.mahalanobis_d2 = float("nan")
        self.innovation = np.array([float("nan"), float("nan"), float("nan"), float("nan")], dtype=float)
        self.observation_confidence = 0.0
        self.accepted = False
        self.pending = False
        self.rejected = False


class TagBoardPoseEstimator:
    def __init__(self, source_video_path):
        self.enabled = _bool_param("enable_camera_trajectory", True)
        self.board_frame = _param("board_frame", "tag1_board")
        self.csv_path = str(_param("camera_trajectory_csv_path", "")).strip() or _derive_trajectory_path(source_video_path)

        self.filter_enabled = _bool_param("filter_enabled", True)
        self.use_rts_smoother = _bool_param("use_rts_smoother", True)
        self.use_interpolation = _bool_param("use_interpolation", True)
        self.sigma_a_xy = max(1.0e-6, float(_param("sigma_a_xy", 1.0)))
        self.sigma_a_z = max(1.0e-6, float(_param("sigma_a_z", 0.8)))
        self.sigma_yaw_acc = max(1.0e-6, float(_param("sigma_yaw_acc", 0.8)))
        self.confidence_min = _clamp(float(_param("confidence_min", 0.1)), 1.0e-3, 1.0)
        self.min_accept_confidence = _clamp(float(_param("min_accept_confidence", 0.2)), 0.0, 1.0)
        self.r_base_position = max(1.0e-6, float(_param("R_base_position", 0.03)))
        self.r_base_yaw = max(1.0e-6, float(_param("R_base_yaw", 0.05)))
        self.mahalanobis_threshold = max(0.1, float(_param("mahalanobis_threshold", 13.0)))
        self.pending_confirm_count = max(1, int(_param("pending_confirm_count", 3)))
        self.short_lost_time = max(0.0, float(_param("short_lost_time", 0.3)))
        self.mid_lost_time = max(self.short_lost_time, float(_param("mid_lost_time", 1.0)))
        self.max_dt = max(0.001, float(_param("max_dt", 0.5)))
        self.max_position_jump = max(0.001, float(_param("max_position_jump", 0.5)))
        self.max_horizontal_speed = max(0.001, float(_param("max_horizontal_speed", 2.0)))
        self.max_vertical_speed = max(0.001, float(_param("max_vertical_speed", 1.5)))
        self.max_yaw_jump = math.radians(max(0.1, float(_param("max_yaw_jump_deg", 20.0))))
        self.max_reprojection_error = max(0.1, float(_param("max_reprojection_error", 5.0)))
        self.min_border_distance = max(0.0, float(_param("min_border_distance", 10.0)))

        self.pose_outlier_distance = max(0.0, float(_param("pose_outlier_distance", 0.25)))
        self.pose_outlier_angle = max(0.0, float(_param("pose_outlier_angle", 0.8)))
        self.trajectory_stable_frame_threshold = max(1, int(_param("trajectory_stable_frame_threshold", 3)))
        self.single_tag_confidence_scale = min(1.0, max(0.0, float(_param("single_tag_confidence_scale", 0.55))))
        self.prediction_confidence_scale = min(1.0, max(0.0, float(_param("prediction_confidence_scale", 0.35))))

        self.tag_layout = {}
        self.load_tag_layout()

        self.camera_width = 0
        self.camera_height = 0
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

        self.kf_initialized = False
        self.x = np.zeros(8, dtype=float)
        self.P = np.eye(8, dtype=float)
        self.last_filter_time = None
        self.last_accepted_time = None
        self.stable_count = 0
        self.lost_count = 0
        self.pending_observations = []
        self.records = []
        self.last_result = CameraTrajectoryResult()

        self.csv_file = None
        self.csv_writer = None

    def load_tag_layout(self):
        for tag_id in [1, 2, 3]:
            xyz_default = {
                1: [0.0, 0.0, 0.0],
                2: [0.0, 0.15, -0.12],
                3: [0.0, -0.15, -0.12],
            }[tag_id]
            rpy_default = {
                1: [0.0, 0.0, 0.0],
                2: [0.0, 0.0, 0.0],
                3: [0.0, 0.0, math.pi],
            }[tag_id]
            xyz = _parse_vector3_param("tag_{}_board_xyz".format(tag_id), xyz_default)
            rpy = _parse_vector3_param("tag_{}_board_rpy".format(tag_id), rpy_default)
            transform = _make_transform(_rpy_to_matrix(rpy[0], rpy[1], rpy[2]), xyz)
            self.tag_layout[tag_id] = {
                "xyz": xyz,
                "rpy": rpy,
                "transform": transform,
            }

    def configure_camera(self, width, height, fx, fy, cx, cy):
        self.camera_width = int(width)
        self.camera_height = int(height)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)

    def open_csv(self):
        if not self.enabled:
            return
        output_dir = os.path.dirname(self.csv_path)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "frame_index",
                "stamp_sec",
                "state",
                "valid",
                "confidence",
                "used_tag_ids",
                "rejected_tag_ids",
                "camera_x",
                "camera_y",
                "camera_z",
                "camera_roll",
                "camera_pitch",
                "camera_yaw",
                "raw_camera_x",
                "raw_camera_y",
                "raw_camera_z",
                "residual_position",
                "detection_count",
                "status",
                "mahalanobis_d2",
                "innovation_px",
                "innovation_py",
                "innovation_pz",
                "innovation_yaw",
                "observation_confidence",
                "accepted",
                "pending",
                "rejected",
                "point_source",
            ],
        )
        self.csv_writer.writeheader()
        rospy.loginfo("[VideoCameraPublisher] camera trajectory csv=%s board_frame=%s", self.csv_path, self.board_frame)

    def close_csv(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    def project_camera_point_float(self, point):
        if self.camera_width <= 0 or self.camera_height <= 0:
            return None
        x, y, z = point
        if not all(math.isfinite(v) for v in [x, y, z]) or z <= 1.0e-6:
            return None
        return np.array([
            self.fx * x / z + self.cx,
            self.fy * y / z + self.cy,
        ], dtype=float)

    def estimate_projection_quality(self, camera_tag, tag_size):
        # apriltag_ros does not expose raw image corners or reprojection error in
        # AprilTagDetection. We reproject the solved pose corners to estimate
        # apparent area and border margin. Reprojection error is kept as a
        # protected placeholder because the detector has already solved PnP.
        half = 0.5 * tag_size
        corners_tag = [
            np.array([-half, -half, 0.0, 1.0], dtype=float),
            np.array([half, -half, 0.0, 1.0], dtype=float),
            np.array([half, half, 0.0, 1.0], dtype=float),
            np.array([-half, half, 0.0, 1.0], dtype=float),
        ]
        pixels = []
        for corner in corners_tag:
            point_camera = camera_tag.dot(corner)[:3]
            pixel = self.project_camera_point_float(point_camera)
            if pixel is None or not _finite_array(pixel):
                return {
                    "area": 0.0,
                    "border_distance": 0.0,
                    "reprojection_error": self.max_reprojection_error * 2.0,
                    "projection_valid": False,
                }
            pixels.append(pixel)

        area = _polygon_area(pixels)
        min_border = min(
            min(p[0], self.camera_width - 1.0 - p[0], p[1], self.camera_height - 1.0 - p[1])
            for p in pixels
        )
        return {
            "area": max(0.0, area),
            "border_distance": max(0.0, min_border),
            "reprojection_error": 0.0,
            "projection_valid": True,
        }

    def candidate_confidence(self, candidate):
        distance = max(1.0e-6, candidate["distance"])
        area = candidate["area"]
        border = candidate["border_distance"]
        reprojection_error = candidate["reprojection_error"]

        if candidate["projection_valid"]:
            good_area = max(200.0, 0.0008 * float(max(1, self.camera_width * self.camera_height)))
            area_factor = _clamp(area / good_area, 0.05, 1.0)
            border_factor = 1.0
            if self.min_border_distance > 0.0:
                border_factor = _clamp(border / (4.0 * self.min_border_distance), 0.05, 1.0)
                if border < self.min_border_distance:
                    border_factor *= 0.2
            reproj_factor = math.exp(-reprojection_error / max(1.0e-6, self.max_reprojection_error))
        else:
            area_factor = 0.35
            border_factor = 0.35
            reproj_factor = 0.35

        distance_factor = _clamp(1.0 / (1.0 + 0.12 * distance), 0.25, 1.0)
        size_factor = _clamp(candidate["tag_size"] / 0.18, 0.35, 1.0)
        history_factor = 1.0
        if self.kf_initialized:
            position_delta = float(np.linalg.norm(candidate["position"] - self.x[:3]))
            yaw_delta = abs(_wrap_to_pi(candidate["yaw"] - self.x[6]))
            if position_delta > self.max_position_jump:
                history_factor *= math.exp(-(position_delta - self.max_position_jump) / max(0.05, self.max_position_jump))
            if yaw_delta > self.max_yaw_jump:
                history_factor *= math.exp(-(yaw_delta - self.max_yaw_jump) / max(0.05, self.max_yaw_jump))

        confidence = area_factor * border_factor * reproj_factor * distance_factor * size_factor * history_factor
        return _clamp(confidence, 0.0, 1.0)

    def make_candidate(self, detection):
        if not detection.id:
            return None
        tag_id = int(detection.id[0])
        if tag_id not in self.tag_layout:
            return None

        camera_tag = _pose_to_transform(detection.pose.pose.pose)
        if camera_tag is None:
            return None

        board_tag = self.tag_layout[tag_id]["transform"]
        camera_board = camera_tag.dot(_invert_transform(board_tag))
        board_camera = _invert_transform(camera_board)

        camera_position = board_camera[:3, 3]
        camera_quat = _matrix_to_quaternion(board_camera[:3, :3])
        if camera_quat is None or not np.all(np.isfinite(camera_position)):
            return None
        _, _, yaw = _matrix_to_rpy(board_camera[:3, :3])

        tag_position_camera = np.array([
            detection.pose.pose.pose.position.x,
            detection.pose.pose.pose.position.y,
            detection.pose.pose.pose.position.z,
        ], dtype=float)
        distance = float(np.linalg.norm(tag_position_camera))
        tag_size = float(detection.size[0]) if detection.size else 0.045
        if not math.isfinite(tag_size) or tag_size <= 0.0:
            tag_size = 0.045

        projection = self.estimate_projection_quality(camera_tag, tag_size)
        candidate = {
            "tag_id": tag_id,
            "position": camera_position,
            "quat": camera_quat,
            "yaw": _wrap_to_pi(yaw),
            "distance": distance,
            "tag_size": tag_size,
            "area": projection["area"],
            "border_distance": projection["border_distance"],
            "reprojection_error": projection["reprojection_error"],
            "projection_valid": projection["projection_valid"],
        }
        candidate["confidence"] = self.candidate_confidence(candidate)
        candidate["weight"] = max(1.0e-6, candidate["confidence"] * (tag_size * tag_size) / max(distance * distance, 1.0e-4))
        return candidate

    def fuse_candidates(self, candidates):
        result = CameraTrajectoryResult()
        result.detection_count = len(candidates)
        if not candidates:
            return result, None

        candidates = [c for c in candidates if c["confidence"] >= self.min_accept_confidence * 0.5]
        if not candidates:
            result.rejected_tag_ids = [int(c["tag_id"]) for c in candidates]
            return result, None

        positions = np.array([candidate["position"] for candidate in candidates], dtype=float)
        median_position = np.median(positions, axis=0)
        best_reference = None
        if self.kf_initialized:
            best_reference = self.x[:3].copy()
        elif len(candidates) > 0:
            best_reference = median_position

        scored = []
        for candidate in candidates:
            consistency = 1.0
            for other in candidates:
                if other is candidate:
                    continue
                pos_delta = float(np.linalg.norm(candidate["position"] - other["position"]))
                yaw_delta = abs(_wrap_to_pi(candidate["yaw"] - other["yaw"]))
                if pos_delta > 0.3 or yaw_delta > math.radians(20.0):
                    consistency *= 0.45
            continuity = 1.0
            if best_reference is not None:
                pos_delta = float(np.linalg.norm(candidate["position"] - best_reference))
                continuity *= math.exp(-max(0.0, pos_delta - self.max_position_jump) / max(0.05, self.max_position_jump))
            scored.append((candidate, candidate["confidence"] * consistency * continuity))

        used = []
        rejected = []
        max_score = max(score for _, score in scored) if scored else 0.0
        for candidate, score in scored:
            if len(candidates) > 1 and score < max(0.05, max_score * 0.45):
                rejected.append(candidate)
            else:
                used.append((candidate, max(1.0e-6, score)))

        if not used and scored:
            candidate, score = max(scored, key=lambda item: item[1])
            used = [(candidate, max(1.0e-6, score))]
            rejected = [c for c, _ in scored if c is not candidate]

        weights = np.array([score for _, score in used], dtype=float)
        weights = weights / max(1.0e-9, float(np.sum(weights)))
        fused_position = np.zeros(3, dtype=float)
        sin_yaw = 0.0
        cos_yaw = 0.0
        used_candidates = [candidate for candidate, _ in used]
        for candidate, weight in zip(used_candidates, weights):
            fused_position += candidate["position"] * weight
            sin_yaw += weight * math.sin(candidate["yaw"])
            cos_yaw += weight * math.cos(candidate["yaw"])
        fused_yaw = _wrap_to_pi(math.atan2(sin_yaw, cos_yaw))

        residual = 0.0
        if len(used_candidates) > 1:
            residual = max(float(np.linalg.norm(candidate["position"] - fused_position)) for candidate in used_candidates)

        if len(used_candidates) == 1:
            tag_factor = self.single_tag_confidence_scale
        elif len(used_candidates) == 2:
            tag_factor = 0.8
        else:
            tag_factor = 1.0
        residual_factor = math.exp(-residual / max(0.05, self.pose_outlier_distance))
        confidence = float(np.sum([candidate["confidence"] for candidate in used_candidates]) / max(1, len(used_candidates)))
        confidence = _clamp(confidence * tag_factor * residual_factor, 0.0, 1.0)

        result.raw_camera_position = fused_position.copy()
        result.used_tag_ids = [candidate["tag_id"] for candidate in used_candidates]
        result.rejected_tag_ids = [candidate["tag_id"] for candidate in rejected]
        result.residual_position = residual
        result.observation_confidence = confidence
        observation = {
            "position": fused_position,
            "yaw": fused_yaw,
            "confidence": confidence,
            "residual": residual,
            "used_tag_ids": result.used_tag_ids,
            "rejected_tag_ids": result.rejected_tag_ids,
        }
        return result, observation

    def make_F_Q(self, dt):
        dt = _clamp(dt, 1.0e-4, self.max_dt)
        F = np.eye(8, dtype=float)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[6, 7] = dt

        Q = np.zeros((8, 8), dtype=float)

        def fill_axis(position_index, velocity_index, sigma):
            q = sigma * sigma
            Q[position_index, position_index] = q * dt ** 4 / 4.0
            Q[position_index, velocity_index] = q * dt ** 3 / 2.0
            Q[velocity_index, position_index] = q * dt ** 3 / 2.0
            Q[velocity_index, velocity_index] = q * dt ** 2

        fill_axis(0, 3, self.sigma_a_xy)
        fill_axis(1, 4, self.sigma_a_xy)
        fill_axis(2, 5, self.sigma_a_z)
        fill_axis(6, 7, self.sigma_yaw_acc)
        return F, Q

    def predict_state(self, video_time, source_fps):
        nominal_dt = 1.0 / max(1.0, source_fps)
        if not self.kf_initialized or self.last_filter_time is None:
            return None, None, np.eye(8, dtype=float), nominal_dt
        dt = video_time - self.last_filter_time
        if not math.isfinite(dt) or dt <= 0.0:
            dt = nominal_dt
        dt = min(dt, self.max_dt)
        F, Q = self.make_F_Q(dt)
        x_pred = F.dot(self.x)
        x_pred[6] = _wrap_to_pi(x_pred[6])
        P_pred = F.dot(self.P).dot(F.T) + Q
        return x_pred, P_pred, F, dt

    def initialize_filter(self, observation, video_time):
        self.kf_initialized = True
        self.x = np.zeros(8, dtype=float)
        self.x[:3] = observation["position"]
        self.x[6] = observation["yaw"]
        self.P = np.diag([
            self.r_base_position,
            self.r_base_position,
            self.r_base_position,
            1.0,
            1.0,
            1.0,
            self.r_base_yaw,
            1.0,
        ]).astype(float)
        self.last_filter_time = video_time
        self.last_accepted_time = video_time
        self.stable_count = 1
        self.lost_count = 0
        self.pending_observations = []

    def result_from_state(self, result, x_state, state, valid, confidence, point_source):
        result.state = state
        result.status = state
        result.valid = valid
        result.confidence = _clamp(confidence, 0.0, 1.0)
        result.point_source = point_source
        result.camera_position = x_state[:3].copy()
        result.camera_rpy = np.array([0.0, 0.0, _wrap_to_pi(x_state[6])], dtype=float)
        result.camera_quat = _yaw_to_quat(x_state[6])
        return result

    def make_record(self, frame_index, video_time, result, x_pred=None, P_pred=None, x_filt=None, P_filt=None, F=None, has_state=False):
        return {
            "frame_index": frame_index,
            "stamp_sec": video_time,
            "result": result,
            "x_pred": None if x_pred is None else x_pred.copy(),
            "P_pred": None if P_pred is None else P_pred.copy(),
            "x_filt": None if x_filt is None else x_filt.copy(),
            "P_filt": None if P_filt is None else P_filt.copy(),
            "F": None if F is None else F.copy(),
            "has_state": bool(has_state),
            "final_x": None,
            "final_source": result.point_source,
        }

    def append_record(self, record):
        self.records.append(record)

    def update_pending(self, observation, result, video_time):
        self.pending_observations.append(observation)
        self.pending_observations = self.pending_observations[-self.pending_confirm_count:]
        result.pending = True
        result.point_source = "PENDING"
        result.status = "PENDING"

        if len(self.pending_observations) < self.pending_confirm_count:
            return False

        first = self.pending_observations[0]
        last = self.pending_observations[-1]
        if float(np.linalg.norm(last["position"] - first["position"])) > self.max_position_jump:
            return False
        if abs(_wrap_to_pi(last["yaw"] - first["yaw"])) > self.max_yaw_jump:
            return False

        self.initialize_filter(last, video_time)
        result.accepted = True
        result.pending = False
        result.status = "DETECTED"
        result.point_source = "FILTERED"
        return True

    def process_observation(self, result, observation, video_time, source_fps):
        result.raw_camera_position = observation["position"].copy()
        result.used_tag_ids = observation["used_tag_ids"]
        result.rejected_tag_ids = observation["rejected_tag_ids"]
        result.residual_position = observation["residual"]
        result.observation_confidence = observation["confidence"]

        if observation["confidence"] < self.min_accept_confidence:
            result.rejected = True
            result.status = "REJECTED"
            result.point_source = "REJECTED"
            return result, self.make_record(len(self.records) + 1, video_time, result, has_state=False)

        if not self.filter_enabled:
            x_direct = np.zeros(8, dtype=float)
            x_direct[:3] = observation["position"]
            x_direct[6] = observation["yaw"]
            self.initialize_filter(observation, video_time)
            result.accepted = True
            result = self.result_from_state(result, x_direct, "TRACKING", True, observation["confidence"], "FILTERED")
            return result, self.make_record(len(self.records) + 1, video_time, result, x_filt=x_direct, P_filt=self.P, F=np.eye(8), has_state=True)

        if not self.kf_initialized:
            self.initialize_filter(observation, video_time)
            result.accepted = True
            result = self.result_from_state(result, self.x, "DETECTED", True, observation["confidence"], "FILTERED")
            return result, self.make_record(len(self.records) + 1, video_time, result, x_pred=self.x, P_pred=self.P, x_filt=self.x, P_filt=self.P, F=np.eye(8), has_state=True)

        x_pred, P_pred, F, _ = self.predict_state(video_time, source_fps)
        H = np.zeros((4, 8), dtype=float)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        H[3, 6] = 1.0

        z = np.array([
            observation["position"][0],
            observation["position"][1],
            observation["position"][2],
            observation["yaw"],
        ], dtype=float)
        innovation = z - H.dot(x_pred)
        innovation[3] = _wrap_to_pi(innovation[3])
        result.innovation = innovation.copy()

        conf = max(self.confidence_min, observation["confidence"])
        R = np.diag([
            self.r_base_position / conf,
            self.r_base_position / conf,
            self.r_base_position / conf,
            self.r_base_yaw / conf,
        ]).astype(float)
        S = H.dot(P_pred).dot(H.T) + R
        S_inv = _safe_inverse(S)
        mahalanobis_d2 = float(innovation.T.dot(S_inv).dot(innovation))
        result.mahalanobis_d2 = mahalanobis_d2

        severe_threshold = self.mahalanobis_threshold * 4.0
        if mahalanobis_d2 <= self.mahalanobis_threshold:
            K = P_pred.dot(H.T).dot(S_inv)
            self.x = x_pred + K.dot(innovation)
            self.x[6] = _wrap_to_pi(self.x[6])
            I = np.eye(8, dtype=float)
            self.P = (I - K.dot(H)).dot(P_pred)
            self.last_filter_time = video_time
            self.last_accepted_time = video_time
            self.stable_count += 1
            self.lost_count = 0
            self.pending_observations = []
            result.accepted = True
            state = "TRACKING" if self.stable_count >= self.trajectory_stable_frame_threshold else "DETECTED"
            result = self.result_from_state(result, self.x, state, True, observation["confidence"], "FILTERED")
            return result, self.make_record(len(self.records) + 1, video_time, result, x_pred, P_pred, self.x, self.P, F, True)

        if mahalanobis_d2 <= severe_threshold and self.update_pending(observation, result, video_time):
            result = self.result_from_state(result, self.x, "DETECTED", True, observation["confidence"] * 0.7, "FILTERED")
            return result, self.make_record(len(self.records) + 1, video_time, result, x_pred, P_pred, self.x, self.P, F, True)

        result.rejected = True
        result.status = "REJECTED"
        result.state = "REJECTED"
        result.valid = False
        result.confidence = 0.0
        result.point_source = "REJECTED" if mahalanobis_d2 > severe_threshold else "PENDING"
        return result, self.make_record(len(self.records) + 1, video_time, result, x_pred, P_pred, has_state=False)

    def process_missing(self, result, video_time, source_fps):
        self.lost_count += 1
        if not self.kf_initialized or self.last_filter_time is None:
            result.state = "NO_TARGET"
            result.status = "NO_TARGET"
            result.valid = False
            result.point_source = "LOST"
            return result, self.make_record(len(self.records) + 1, video_time, result, has_state=False)

        lost_reference = self.last_accepted_time if self.last_accepted_time is not None else self.last_filter_time
        lost_time = max(0.0, video_time - lost_reference)
        x_pred, P_pred, F, _ = self.predict_state(video_time, source_fps)

        if lost_time <= self.short_lost_time:
            self.x = x_pred
            self.P = P_pred
            self.last_filter_time = video_time
            decay = math.exp(-lost_time / max(1.0e-6, self.short_lost_time))
            confidence = self.prediction_confidence_scale * decay
            result = self.result_from_state(result, self.x, "PREDICTING", True, confidence, "PREDICTED")
            return result, self.make_record(len(self.records) + 1, video_time, result, x_pred, P_pred, self.x, self.P, F, True)

        # Keep the internal filter clock moving, but do not expose a long pure
        # prediction as a valid point. Medium gaps may be filled later for display only.
        self.x = x_pred
        self.P = P_pred
        self.last_filter_time = video_time
        result = self.result_from_state(result, self.x, "LOST", False, 0.0, "LOST")
        return result, self.make_record(len(self.records) + 1, video_time, result, x_pred, P_pred, has_state=False)

    def update(self, detection_msg, frame_index, video_time, source_fps):
        result = CameraTrajectoryResult()
        if not self.enabled:
            return result

        candidates = []
        if detection_msg is not None:
            for detection in detection_msg.detections:
                candidate = self.make_candidate(detection)
                if candidate is not None:
                    candidates.append(candidate)

        fused_result, fused = self.fuse_candidates(candidates)
        result.detection_count = fused_result.detection_count
        result.used_tag_ids = fused_result.used_tag_ids
        result.rejected_tag_ids = fused_result.rejected_tag_ids
        result.raw_camera_position = fused_result.raw_camera_position
        result.residual_position = fused_result.residual_position
        result.observation_confidence = fused_result.observation_confidence

        if fused:
            result, record = self.process_observation(result, fused, video_time, source_fps)
        else:
            result, record = self.process_missing(result, video_time, source_fps)

        self.last_result = result
        record["frame_index"] = frame_index
        record["stamp_sec"] = video_time
        self.append_record(record)
        return result

    def run_rts_smoother(self):
        if not self.use_rts_smoother:
            return

        index = 0
        while index < len(self.records):
            while index < len(self.records) and not self.records[index]["has_state"]:
                index += 1
            start = index
            while index < len(self.records) and self.records[index]["has_state"]:
                index += 1
            end = index
            if end - start <= 1:
                for rec in self.records[start:end]:
                    rec["final_x"] = rec["x_filt"].copy()
                    rec["final_source"] = rec["result"].point_source
                continue

            segment = self.records[start:end]
            segment[-1]["final_x"] = segment[-1]["x_filt"].copy()
            P_smooth = segment[-1]["P_filt"].copy()
            for local_idx in range(len(segment) - 2, -1, -1):
                rec = segment[local_idx]
                next_rec = segment[local_idx + 1]
                P_filt = rec["P_filt"]
                F_next = next_rec["F"] if next_rec["F"] is not None else np.eye(8, dtype=float)
                P_pred_next = next_rec["P_pred"] if next_rec["P_pred"] is not None else next_rec["P_filt"]
                C = P_filt.dot(F_next.T).dot(_safe_inverse(P_pred_next))
                delta = next_rec["final_x"] - next_rec["x_pred"]
                delta[6] = _wrap_to_pi(delta[6])
                smoothed = rec["x_filt"] + C.dot(delta)
                smoothed[6] = _wrap_to_pi(smoothed[6])
                rec["final_x"] = smoothed
                P_smooth = P_filt + C.dot(P_smooth - P_pred_next).dot(C.T)
                rec["final_source"] = rec["result"].point_source

    def seed_forward_results_as_final(self):
        for rec in self.records:
            if rec["has_state"] and rec["final_x"] is None and rec["x_filt"] is not None:
                rec["final_x"] = rec["x_filt"].copy()
                rec["final_source"] = rec["result"].point_source

    def interpolate_gaps(self):
        if not self.use_interpolation:
            return

        idx = 0
        while idx < len(self.records):
            if self.records[idx]["final_x"] is not None:
                idx += 1
                continue
            gap_start = idx
            while idx < len(self.records) and self.records[idx]["final_x"] is None:
                idx += 1
            gap_end = idx
            prev_idx = gap_start - 1
            next_idx = gap_end
            if prev_idx < 0 or next_idx >= len(self.records):
                continue
            prev_rec = self.records[prev_idx]
            next_rec = self.records[next_idx]
            if prev_rec["final_x"] is None or next_rec["final_x"] is None:
                continue
            duration = next_rec["stamp_sec"] - prev_rec["stamp_sec"]
            if duration <= 0.0 or duration > self.mid_lost_time:
                continue

            delta_pos = next_rec["final_x"][:3] - prev_rec["final_x"][:3]
            horizontal_speed = float(np.linalg.norm(delta_pos[:2])) / duration
            vertical_speed = abs(float(delta_pos[2])) / duration
            if horizontal_speed > self.max_horizontal_speed or vertical_speed > self.max_vertical_speed:
                continue

            yaw0 = prev_rec["final_x"][6]
            yaw_delta = _wrap_to_pi(next_rec["final_x"][6] - yaw0)
            for gap_idx in range(gap_start, gap_end):
                rec = self.records[gap_idx]
                ratio = (rec["stamp_sec"] - prev_rec["stamp_sec"]) / duration
                interp = prev_rec["final_x"].copy()
                interp[:3] = prev_rec["final_x"][:3] + ratio * delta_pos
                interp[3:6] = delta_pos / duration
                interp[6] = _wrap_to_pi(yaw0 + ratio * yaw_delta)
                interp[7] = yaw_delta / duration
                rec["final_x"] = interp
                rec["final_source"] = "INTERPOLATED"

    def finalize_offline(self):
        if not self.enabled:
            return []
        self.seed_forward_results_as_final()
        self.run_rts_smoother()
        self.interpolate_gaps()
        self.write_csv_rows()
        return self.records

    def write_csv_rows(self):
        if self.csv_writer is None:
            return
        for rec in self.records:
            result = rec["result"]
            final_x = rec["final_x"]
            if final_x is not None:
                camera_position = final_x[:3]
                camera_rpy = np.array([0.0, 0.0, _wrap_to_pi(final_x[6])], dtype=float)
                valid = 1
            else:
                camera_position = result.camera_position
                camera_rpy = result.camera_rpy
                valid = int(result.valid)
            self.csv_writer.writerow({
                "frame_index": rec["frame_index"],
                "stamp_sec": "{:.6f}".format(rec["stamp_sec"]),
                "state": result.state,
                "valid": valid,
                "confidence": "{:.6f}".format(result.confidence),
                "used_tag_ids": _format_id_list(result.used_tag_ids),
                "rejected_tag_ids": _format_id_list(result.rejected_tag_ids),
                "camera_x": "{:.6f}".format(_safe_float(camera_position[0])),
                "camera_y": "{:.6f}".format(_safe_float(camera_position[1])),
                "camera_z": "{:.6f}".format(_safe_float(camera_position[2])),
                "camera_roll": "{:.6f}".format(_safe_float(camera_rpy[0])),
                "camera_pitch": "{:.6f}".format(_safe_float(camera_rpy[1])),
                "camera_yaw": "{:.6f}".format(_safe_float(camera_rpy[2])),
                "raw_camera_x": "{:.6f}".format(_safe_float(result.raw_camera_position[0])),
                "raw_camera_y": "{:.6f}".format(_safe_float(result.raw_camera_position[1])),
                "raw_camera_z": "{:.6f}".format(_safe_float(result.raw_camera_position[2])),
                "residual_position": "{:.6f}".format(_safe_float(result.residual_position)),
                "detection_count": result.detection_count,
                "status": result.status,
                "mahalanobis_d2": "{:.6f}".format(_safe_float(result.mahalanobis_d2)),
                "innovation_px": "{:.6f}".format(_safe_float(result.innovation[0])),
                "innovation_py": "{:.6f}".format(_safe_float(result.innovation[1])),
                "innovation_pz": "{:.6f}".format(_safe_float(result.innovation[2])),
                "innovation_yaw": "{:.6f}".format(_safe_float(result.innovation[3])),
                "observation_confidence": "{:.6f}".format(result.observation_confidence),
                "accepted": int(result.accepted),
                "pending": int(result.pending),
                "rejected": int(result.rejected),
                "point_source": rec["final_source"],
            })


def _quat_to_matrix_np(quat):
    quat = _quat_normalize(quat)
    if quat is None:
        return np.eye(3, dtype=float)
    x, y, z, w = quat
    return np.array(_quaternion_to_matrix(type("Q", (), {"x": x, "y": y, "z": z, "w": w})()), dtype=float)


def _make_point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


class CameraTrajectoryRvizPublisher:
    def __init__(self, frame_id):
        self.enabled = _bool_param("publish_rviz_trajectory", True)
        self.frame_id = frame_id
        self.pose_topic = _param("rviz_trajectory_pose_topic", "/uav/sensing/camera_trajectory/pose")
        self.path_topic = _param("rviz_trajectory_path_topic", "/uav/sensing/camera_trajectory/path")
        self.marker_topic = _param("rviz_trajectory_marker_topic", "/uav/sensing/camera_trajectory/markers")
        self.path_max_points = max(1, int(_param("rviz_path_max_points", 5000)))
        self.axis_length = max(0.01, float(_param("rviz_axis_length", 0.15)))
        self.path = Path()
        self.path.header.frame_id = self.frame_id

        if self.enabled:
            self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
            self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
            self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
            rospy.loginfo(
                "[VideoCameraPublisher] rviz trajectory pose=%s path=%s markers=%s fixed_frame=%s",
                self.pose_topic,
                self.path_topic,
                self.marker_topic,
                self.frame_id,
            )
        else:
            self.pose_pub = None
            self.path_pub = None
            self.marker_pub = None

    def make_pose_msg(self, stamp, result):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = _safe_float(result.camera_position[0])
        pose_msg.pose.position.y = _safe_float(result.camera_position[1])
        pose_msg.pose.position.z = _safe_float(result.camera_position[2])
        quat = result.camera_quat if result.camera_quat is not None else np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        quat = _quat_normalize(quat)
        if quat is None:
            quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        return pose_msg

    def make_marker(self, stamp, marker_id, marker_type, ns, lifetime=0.5):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(lifetime)
        return marker

    def append_arrow_marker(self, markers, stamp, marker_id, ns, start, end, color):
        marker = self.make_marker(stamp, marker_id, Marker.ARROW, ns)
        marker.points = [
            _make_point(start[0], start[1], start[2]),
            _make_point(end[0], end[1], end[2]),
        ]
        marker.scale.x = 0.01
        marker.scale.y = 0.025
        marker.scale.z = 0.04
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        markers.markers.append(marker)

    def append_board_axes(self, markers, stamp):
        origin = np.zeros(3, dtype=float)
        length = self.axis_length
        self.append_arrow_marker(markers, stamp, 10, "board_axes", origin, origin + np.array([length, 0.0, 0.0]), (1.0, 0.0, 0.0))
        self.append_arrow_marker(markers, stamp, 11, "board_axes", origin, origin + np.array([0.0, length, 0.0]), (0.0, 1.0, 0.0))
        self.append_arrow_marker(markers, stamp, 12, "board_axes", origin, origin + np.array([0.0, 0.0, length]), (0.0, 0.2, 1.0))

    def append_camera_markers(self, markers, stamp, result):
        if not result.valid or not np.all(np.isfinite(result.camera_position)):
            return

        sphere = self.make_marker(stamp, 20, Marker.SPHERE, "camera")
        sphere.pose.position.x = float(result.camera_position[0])
        sphere.pose.position.y = float(result.camera_position[1])
        sphere.pose.position.z = float(result.camera_position[2])
        sphere.scale.x = 0.045
        sphere.scale.y = 0.045
        sphere.scale.z = 0.045
        sphere.color.r = 1.0 - result.confidence
        sphere.color.g = result.confidence
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        markers.markers.append(sphere)

        rotation = _quat_to_matrix_np(result.camera_quat)
        origin = result.camera_position
        length = self.axis_length * 0.8
        self.append_arrow_marker(markers, stamp, 21, "camera_axes", origin, origin + rotation[:, 0] * length, (1.0, 0.0, 0.0))
        self.append_arrow_marker(markers, stamp, 22, "camera_axes", origin, origin + rotation[:, 1] * length, (0.0, 1.0, 0.0))
        self.append_arrow_marker(markers, stamp, 23, "camera_axes", origin, origin + rotation[:, 2] * length, (0.0, 0.2, 1.0))

        text = self.make_marker(stamp, 24, Marker.TEXT_VIEW_FACING, "camera_text")
        text.pose.position.x = float(origin[0])
        text.pose.position.y = float(origin[1])
        text.pose.position.z = float(origin[2] + 0.08)
        text.scale.z = 0.04
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "{} conf={:.2f} tags={}".format(
            result.state,
            result.confidence,
            ",".join(str(x) for x in result.used_tag_ids) if result.used_tag_ids else "-",
        )
        markers.markers.append(text)

    def append_path_marker(self, markers, stamp):
        if not self.path.poses:
            return
        marker = self.make_marker(stamp, 30, Marker.LINE_STRIP, "camera_path")
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = [
            _make_point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in self.path.poses
        ]
        markers.markers.append(marker)

    def publish(self, stamp, result):
        if not self.enabled:
            return

        markers = MarkerArray()
        self.append_board_axes(markers, stamp)

        if result.valid:
            pose_msg = self.make_pose_msg(stamp, result)
            self.pose_pub.publish(pose_msg)
            self.path.header.stamp = stamp
            self.path.header.frame_id = self.frame_id
            self.path.poses.append(pose_msg)
            if len(self.path.poses) > self.path_max_points:
                self.path.poses = self.path.poses[-self.path_max_points:]
            self.path_pub.publish(self.path)

        self.append_path_marker(markers, stamp)
        self.append_camera_markers(markers, stamp, result)
        self.marker_pub.publish(markers)

    def state_to_pose_msg(self, stamp, state):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = float(state[0])
        pose_msg.pose.position.y = float(state[1])
        pose_msg.pose.position.z = float(state[2])
        quat = _yaw_to_quat(state[6])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        return pose_msg

    def append_points_marker(self, markers, stamp, marker_id, ns, points, color, scale=0.035):
        if not points:
            return
        marker = self.make_marker(stamp, marker_id, Marker.SPHERE_LIST, ns, lifetime=0.0)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [_make_point(p[0], p[1], p[2]) for p in points]
        markers.markers.append(marker)

    def append_line_marker(self, markers, stamp, marker_id, ns, points, color, width=0.01):
        if len(points) < 2:
            return
        marker = self.make_marker(stamp, marker_id, Marker.LINE_STRIP, ns, lifetime=0.0)
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [_make_point(p[0], p[1], p[2]) for p in points]
        markers.markers.append(marker)

    def publish_final(self, stamp, records):
        if not self.enabled:
            return

        final_path = Path()
        final_path.header.stamp = stamp
        final_path.header.frame_id = self.frame_id
        final_points = []
        raw_points = []
        filtered_points = []
        rejected_points = []
        predicted_points = []
        interpolated_points = []

        for record in records:
            result = record["result"]
            if np.all(np.isfinite(result.raw_camera_position)):
                raw_points.append(result.raw_camera_position)
            if record["x_filt"] is not None:
                filtered_points.append(record["x_filt"][:3])
            if result.rejected and np.all(np.isfinite(result.raw_camera_position)):
                rejected_points.append(result.raw_camera_position)
            if record["final_x"] is not None:
                final_points.append(record["final_x"][:3])
                final_path.poses.append(self.state_to_pose_msg(stamp, record["final_x"]))
                if record["final_source"] == "PREDICTED":
                    predicted_points.append(record["final_x"][:3])
                elif record["final_source"] == "INTERPOLATED":
                    interpolated_points.append(record["final_x"][:3])

        self.path = final_path
        self.path_pub.publish(final_path)

        markers = MarkerArray()
        self.append_board_axes(markers, stamp)
        self.append_line_marker(markers, stamp, 100, "final_path", final_points, (0.0, 1.0, 0.0, 1.0), width=0.018)
        self.append_line_marker(markers, stamp, 101, "raw_path", raw_points, (1.0, 0.6, 0.0, 0.45), width=0.006)
        self.append_line_marker(markers, stamp, 102, "filtered_path", filtered_points, (0.0, 0.5, 1.0, 0.65), width=0.01)
        self.append_points_marker(markers, stamp, 110, "rejected_points", rejected_points, (1.0, 0.0, 0.0, 1.0), scale=0.045)
        self.append_points_marker(markers, stamp, 111, "predicted_points", predicted_points, (0.0, 0.6, 1.0, 1.0), scale=0.035)
        self.append_points_marker(markers, stamp, 112, "interpolated_points", interpolated_points, (1.0, 0.0, 1.0, 1.0), scale=0.035)
        if records and records[-1]["final_x"] is not None:
            final_result = CameraTrajectoryResult()
            final_result.valid = True
            final_result.confidence = records[-1]["result"].confidence
            final_result.state = records[-1]["final_source"]
            final_result.camera_position = records[-1]["final_x"][:3]
            final_result.camera_rpy = np.array([0.0, 0.0, records[-1]["final_x"][6]], dtype=float)
            final_result.camera_quat = _yaw_to_quat(records[-1]["final_x"][6])
            final_result.used_tag_ids = records[-1]["result"].used_tag_ids
            self.append_camera_markers(markers, stamp, final_result)
        self.marker_pub.publish(markers)


class OfflineAprilTagVideoExporter:
    def __init__(self):
        default_video = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            "docs",
            "text.mp4",
        )
        self.video_path = _param("video_path", default_video)
        self.output_path = str(_param("output_path", "")).strip() or _derive_output_path(self.video_path)
        self.trajectory_estimator = TagBoardPoseEstimator(self.video_path)
        self.rviz_publisher = CameraTrajectoryRvizPublisher(self.trajectory_estimator.board_frame)
        self.draw_trajectory_overlay = _bool_param("draw_trajectory_overlay", True)

        self.image_topic = _param("image_topic", "/uav_1/usb_cam/image_raw")
        self.camera_info_topic = _param("camera_info_topic", "/uav_1/usb_cam/camera_info")
        self.tag_detection_topic = _param("tag_detection_topic", "/tag_detections")
        self.frame_id = _param("frame_id", "robot_camera_link")

        self.target_width = int(_param("width", 1920))
        self.target_height = int(_param("height", 1080))
        self.fx = float(_param("fx", 1431.89))
        self.fy = float(_param("fy", 1431.89))
        self.cx = float(_param("cx", 960.0))
        self.cy = float(_param("cy", 540.0))
        self.keep_small_video_native = _bool_param("keep_small_video_native", True)

        self.detection_timeout = float(_param("detection_timeout", 2.0))
        self.sync_by_stamp = _bool_param("sync_by_stamp", True)
        self.sync_tolerance = float(_param("sync_tolerance", 0.001))
        self.fallback_to_raw_on_timeout = _bool_param("fallback_to_raw_on_timeout", True)
        self.wait_for_subscribers = _bool_param("wait_for_subscribers", True)
        self.subscriber_wait_timeout = float(_param("subscriber_wait_timeout", 10.0))
        self.progress_log_interval = float(_param("progress_log_interval", 1.0))
        self.fourcc = str(_param("fourcc", "auto"))
        self.draw_custom_overlay = _bool_param("draw_custom_overlay", True)
        self.overlay_line_thickness = max(1, int(_param("overlay_line_thickness", 4)))
        self.overlay_font_scale = max(0.1, float(_param("overlay_font_scale", 1.0)))
        self.overlay_show_axes = _bool_param("overlay_show_axes", True)
        self.overlay_axis_length_scale = max(0.0, float(_param("overlay_axis_length_scale", 0.5)))
        self.overlay_status_text = _bool_param("overlay_status_text", True)
        self.overlay_timeout_text = _bool_param("overlay_timeout_text", True)
        self.fallback_tag_size = max(0.001, float(_param("fallback_tag_size", 0.20)))

        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.detection_queue = deque(maxlen=32)
        self.writer = None
        self.timeout_count = 0
        self.empty_detection_count = 0
        self.overlay_detection_count = 0
        self.skipped_detection_count = 0
        self.written_count = 0
        self.processed_count = 0
        self.last_progress_time = 0.0
        self.start_time = 0.0

        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=1)
        self.detection_sub = rospy.Subscriber(
            self.tag_detection_topic,
            AprilTagDetectionArray,
            self.detection_array_cb,
            queue_size=32,
        )

    def detection_array_cb(self, msg):
        with self.condition:
            self.detection_queue.append(msg)
            self.condition.notify_all()

    def wait_for_subscribers_ready(self):
        if not self.wait_for_subscribers:
            return

        deadline = time.monotonic() + max(0.0, self.subscriber_wait_timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.image_pub.get_num_connections() > 0 and self.camera_info_pub.get_num_connections() > 0:
                return
            rospy.sleep(0.05)

        rospy.logwarn(
            "[VideoCameraPublisher] subscriber wait timeout; image connections=%d camera_info connections=%d",
            self.image_pub.get_num_connections(),
            self.camera_info_pub.get_num_connections(),
        )

    def prepare_video_geometry(self, cap):
        source_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        source_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        source_fps = float(cap.get(cv2.CAP_PROP_FPS))
        source_frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if not math.isfinite(source_fps) or source_fps <= 0.0:
            source_fps = 30.0

        width = self.target_width
        height = self.target_height
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy

        if width <= 0 or height <= 0:
            width = source_width
            height = source_height
        elif (
            self.keep_small_video_native
            and source_width > 0
            and source_height > 0
            and (source_width < width or source_height < height)
        ):
            scale_x = float(source_width) / float(width)
            scale_y = float(source_height) / float(height)
            width = source_width
            height = source_height
            fx *= scale_x
            fy *= scale_y
            cx *= scale_x
            cy *= scale_y
            rospy.loginfo(
                "[VideoCameraPublisher] source video is smaller than target; keeping native %dx%d resolution",
                width,
                height,
            )

        resize_frames = source_width > 0 and source_height > 0 and (source_width != width or source_height != height)
        if resize_frames:
            rospy.loginfo(
                "[VideoCameraPublisher] resizing frames from %dx%d to %dx%d",
                source_width,
                source_height,
                width,
                height,
            )

        camera_info = _make_camera_info(width, height, self.frame_id, fx, fy, cx, cy)
        return width, height, source_fps, source_frame_count, resize_frames, camera_info

    def open_writer(self, width, height, fps):
        output_dir = os.path.dirname(self.output_path)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        fourcc = _select_fourcc(self.output_path, self.fourcc)
        code = cv2.VideoWriter_fourcc(*fourcc)
        self.writer = cv2.VideoWriter(self.output_path, code, fps, (width, height), True)
        if not self.writer.isOpened():
            raise RuntimeError("failed to open output video: {}".format(self.output_path))

        rospy.loginfo(
            "[VideoCameraPublisher] output=%s size=%dx%d fps=%.3f fourcc=%s",
            self.output_path,
            width,
            height,
            fps,
            fourcc,
        )

    def get_detection_stamp(self, msg):
        if not msg.header.stamp.is_zero():
            return msg.header.stamp
        for detection in msg.detections:
            if not detection.pose.header.stamp.is_zero():
                return detection.pose.header.stamp
        return rospy.Time()

    def find_matching_detection_locked(self, target_stamp):
        if not self.detection_queue:
            return None

        if not self.sync_by_stamp:
            return self.detection_queue.pop()

        for msg in list(self.detection_queue):
            if _time_close(self.get_detection_stamp(msg), target_stamp, self.sync_tolerance):
                self.detection_queue.remove(msg)
                return msg
        return None

    def wait_for_detection_array(self, target_stamp):
        deadline = time.monotonic() + max(0.0, self.detection_timeout)
        with self.condition:
            while not rospy.is_shutdown():
                msg = self.find_matching_detection_locked(target_stamp)
                if msg is not None:
                    return msg

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.condition.wait(timeout=remaining)
        return None

    def project_camera_point(self, point):
        x, y, z = point
        if not all(math.isfinite(v) for v in [x, y, z]) or z <= 1.0e-6:
            return None
        return (
            int(round(self.current_fx * x / z + self.current_cx)),
            int(round(self.current_fy * y / z + self.current_cy)),
        )

    def draw_status_text(self, frame, text, color=(0, 255, 255)):
        if not self.overlay_status_text:
            return
        scale = max(0.5, self.overlay_font_scale)
        thickness = max(1, self.overlay_line_thickness // 2)
        cv2.putText(
            frame,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def draw_axis(self, frame, origin, endpoint, label, color):
        if not (_valid_pixel(origin) and _valid_pixel(endpoint)):
            return
        cv2.line(frame, origin, endpoint, color, self.overlay_line_thickness, cv2.LINE_AA)
        cv2.putText(
            frame,
            label,
            (endpoint[0] + 6, endpoint[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            max(0.4, self.overlay_font_scale * 0.7),
            color,
            max(1, self.overlay_line_thickness // 2),
            cv2.LINE_AA,
        )

    def draw_detection_overlay(self, frame, detection):
        if not detection.id:
            return False

        tag_id = int(detection.id[0])
        tag_size = float(detection.size[0]) if detection.size else self.fallback_tag_size
        if not math.isfinite(tag_size) or tag_size <= 0.0:
            tag_size = self.fallback_tag_size

        pose = detection.pose.pose.pose
        rotation = _quaternion_to_matrix(pose.orientation)
        if rotation is None:
            return False

        translation = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(v) for v in translation):
            return False

        half = 0.5 * tag_size
        corners_tag = [
            (-half, -half, 0.0),
            (half, -half, 0.0),
            (half, half, 0.0),
            (-half, half, 0.0),
        ]
        corners_camera = [_transform_point(rotation, translation, p) for p in corners_tag]
        corners_px = [self.project_camera_point(p) for p in corners_camera]
        if not all(_valid_pixel(p) for p in corners_px):
            return False

        contour = np.array(corners_px, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [contour], True, (0, 255, 255), self.overlay_line_thickness, cv2.LINE_AA)

        center_camera = _transform_point(rotation, translation, (0.0, 0.0, 0.0))
        center_px = self.project_camera_point(center_camera)
        if _valid_pixel(center_px):
            cv2.circle(frame, center_px, max(4, self.overlay_line_thickness + 2), (0, 0, 255), -1, cv2.LINE_AA)

        text_anchor = corners_px[0]
        cv2.putText(
            frame,
            "ID={}".format(tag_id),
            (text_anchor[0], max(25, text_anchor[1] - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.overlay_font_scale,
            (0, 255, 255),
            max(1, self.overlay_line_thickness // 2),
            cv2.LINE_AA,
        )

        if self.overlay_show_axes and self.overlay_axis_length_scale > 0.0 and _valid_pixel(center_px):
            axis_len = tag_size * self.overlay_axis_length_scale
            x_px = self.project_camera_point(_transform_point(rotation, translation, (axis_len, 0.0, 0.0)))
            y_px = self.project_camera_point(_transform_point(rotation, translation, (0.0, axis_len, 0.0)))
            z_px = self.project_camera_point(_transform_point(rotation, translation, (0.0, 0.0, axis_len)))
            self.draw_axis(frame, center_px, x_px, "X", (0, 0, 255))
            self.draw_axis(frame, center_px, y_px, "Y", (0, 255, 0))
            self.draw_axis(frame, center_px, z_px, "Z", (255, 0, 0))

        return True

    def draw_custom_detection_overlay(self, frame, detection_msg):
        output = frame.copy()
        if detection_msg is None:
            if self.overlay_timeout_text:
                self.draw_status_text(output, "DETECTION TIMEOUT", (0, 0, 255))
            return output, 0, 0

        if not detection_msg.detections:
            self.draw_status_text(output, "NO TAG", (0, 255, 255))
            return output, 0, 0

        drawn = 0
        skipped = 0
        for detection in detection_msg.detections:
            if self.draw_detection_overlay(output, detection):
                drawn += 1
            else:
                skipped += 1

        if self.overlay_status_text:
            self.draw_status_text(output, "TAGS={} SKIPPED={}".format(drawn, skipped), (0, 255, 255))
        return output, drawn, skipped

    def draw_camera_trajectory_overlay(self, frame, result):
        if not self.draw_trajectory_overlay or not self.trajectory_estimator.enabled:
            return

        color = (0, 255, 0) if result.valid else (0, 0, 255)
        pos = result.camera_position
        rpy_deg = np.degrees(result.camera_rpy)
        used = ",".join(str(x) for x in result.used_tag_ids) if result.used_tag_ids else "-"

        lines = [
            "CAM {}=({:.3f},{:.3f},{:.3f}) state={} conf={:.2f}".format(
                self.trajectory_estimator.board_frame,
                _safe_float(pos[0]),
                _safe_float(pos[1]),
                _safe_float(pos[2]),
                result.state,
                result.confidence,
            ),
            "RPYdeg=({:.1f},{:.1f},{:.1f}) used={} residual={:.3f}".format(
                _safe_float(rpy_deg[0]),
                _safe_float(rpy_deg[1]),
                _safe_float(rpy_deg[2]),
                used,
                _safe_float(result.residual_position),
            ),
        ]

        scale = max(0.45, self.overlay_font_scale * 0.75)
        thickness = max(1, self.overlay_line_thickness // 2)
        y = 80
        for line in lines:
            cv2.putText(
                frame,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                thickness,
                cv2.LINE_AA,
            )
            y += int(32 * scale) + 8

    def write_frame(self, frame):
        if frame.shape[1] != self.output_width or frame.shape[0] != self.output_height:
            frame = cv2.resize(frame, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)
        self.writer.write(frame)
        self.written_count += 1

    def maybe_log_progress(self, total_frames):
        now_mono = time.monotonic()
        if self.progress_log_interval > 0.0 and (now_mono - self.last_progress_time) < self.progress_log_interval:
            return

        if total_frames > 0:
            progress = min(100.0, 100.0 * float(self.processed_count) / float(total_frames))
            elapsed = max(1.0e-6, now_mono - self.start_time)
            fps_actual = float(self.processed_count) / elapsed if self.processed_count > 0 else 0.0
            remaining = max(0, total_frames - self.processed_count)
            eta = float(remaining) / fps_actual if fps_actual > 0.0 else 0.0
            rospy.loginfo(
                "[VideoCameraPublisher] progress=%.1f%% frame=%d/%d written=%d timeout=%d eta=%.1fs",
                progress,
                self.processed_count,
                total_frames,
                self.written_count,
                self.timeout_count,
                eta,
            )
        else:
            rospy.loginfo(
                "[VideoCameraPublisher] frame=%d written=%d timeout=%d",
                self.processed_count,
                self.written_count,
                self.timeout_count,
            )
        self.last_progress_time = now_mono

    def run(self):
        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            rospy.logerr("[VideoCameraPublisher] failed to open video: %s", self.video_path)
            return

        try:
            (
                self.output_width,
                self.output_height,
                source_fps,
                source_frame_count,
                resize_frames,
                camera_info,
            ) = self.prepare_video_geometry(cap)
            self.open_writer(self.output_width, self.output_height, source_fps)

            rospy.loginfo("[VideoCameraPublisher] video=%s", self.video_path)
            rospy.loginfo("[VideoCameraPublisher] image_topic=%s camera_info_topic=%s", self.image_topic, self.camera_info_topic)
            rospy.loginfo("[VideoCameraPublisher] tag_detection_topic=%s timeout=%.3fs", self.tag_detection_topic, self.detection_timeout)
            rospy.loginfo("[VideoCameraPublisher] custom_overlay=%s thickness=%d axes=%s", self.draw_custom_overlay, self.overlay_line_thickness, self.overlay_show_axes)
            rospy.loginfo("[VideoCameraPublisher] camera_trajectory=%s board_frame=%s", self.trajectory_estimator.enabled, self.trajectory_estimator.board_frame)

            self.wait_for_subscribers_ready()
            self.trajectory_estimator.open_csv()

            self.current_fx = float(camera_info.K[0])
            self.current_fy = float(camera_info.K[4])
            self.current_cx = float(camera_info.K[2])
            self.current_cy = float(camera_info.K[5])
            self.trajectory_estimator.configure_camera(
                self.output_width,
                self.output_height,
                self.current_fx,
                self.current_fy,
                self.current_cx,
                self.current_cy,
            )

            self.start_time = time.monotonic()
            self.last_progress_time = 0.0
            reached_eof = False
            while not rospy.is_shutdown():
                ok, frame = cap.read()
                if not ok:
                    reached_eof = True
                    break

                if resize_frames:
                    interpolation = cv2.INTER_AREA if self.output_width < frame.shape[1] or self.output_height < frame.shape[0] else cv2.INTER_LINEAR
                    frame = cv2.resize(frame, (self.output_width, self.output_height), interpolation=interpolation)

                stamp = rospy.Time.now()
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                image_msg.header.stamp = stamp
                image_msg.header.frame_id = self.frame_id

                camera_info.header.stamp = stamp
                self.camera_info_pub.publish(camera_info)
                self.image_pub.publish(image_msg)

                detection_msg = self.wait_for_detection_array(stamp)
                frame_index = self.processed_count + 1
                video_time = float(self.processed_count) / max(1.0, source_fps)
                trajectory_result = self.trajectory_estimator.update(detection_msg, frame_index, video_time, source_fps)
                self.rviz_publisher.publish(stamp, trajectory_result)
                if detection_msg is None:
                    self.timeout_count += 1
                    if not self.fallback_to_raw_on_timeout:
                        rospy.logwarn("[VideoCameraPublisher] detection timeout at frame %d and fallback disabled", self.processed_count + 1)
                        break
                    if self.draw_custom_overlay:
                        output_frame, drawn, skipped = self.draw_custom_detection_overlay(frame, None)
                    else:
                        output_frame = frame
                        drawn = 0
                        skipped = 0
                else:
                    if self.draw_custom_overlay:
                        output_frame, drawn, skipped = self.draw_custom_detection_overlay(frame, detection_msg)
                        if not detection_msg.detections:
                            self.empty_detection_count += 1
                        self.overlay_detection_count += drawn
                        self.skipped_detection_count += skipped
                    else:
                        output_frame = frame

                self.draw_camera_trajectory_overlay(output_frame, trajectory_result)

                self.processed_count += 1
                self.write_frame(output_frame)
                self.maybe_log_progress(source_frame_count)

            elapsed = max(1.0e-6, time.monotonic() - self.start_time)
            final_records = self.trajectory_estimator.finalize_offline()
            self.rviz_publisher.publish_final(rospy.Time.now(), final_records)
            status = "EXPORT COMPLETE" if reached_eof else "EXPORT INTERRUPTED"
            rospy.loginfo(
                "[VideoCameraPublisher] %s: input=%d written=%d timeout=%d empty=%d drawn=%d skipped=%d output=%s fps=%.3f elapsed=%.1fs",
                status,
                self.processed_count,
                self.written_count,
                self.timeout_count,
                self.empty_detection_count,
                self.overlay_detection_count,
                self.skipped_detection_count,
                self.output_path,
                source_fps,
                elapsed,
            )
            if reached_eof and source_frame_count > 0 and self.written_count != source_frame_count:
                rospy.logwarn(
                    "[VideoCameraPublisher] output frame count differs from source: written=%d source=%d",
                    self.written_count,
                    source_frame_count,
                )
            if final_records:
                rospy.sleep(0.5)
        finally:
            cap.release()
            if self.writer is not None:
                self.writer.release()
            self.trajectory_estimator.close_csv()


def main():
    rospy.init_node("video_camera_publisher")
    OfflineAprilTagVideoExporter().run()


if __name__ == "__main__":
    main()
