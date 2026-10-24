#!/usr/bin/env python3

import os
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _param(name, default):
    return rospy.get_param("~" + name, default)


def _docs_fallback_path():
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "docs",
        "apriltag_detected_output.mp4",
    )


def _derive_output_path(source_video_path):
    source_video_path = str(source_video_path).strip()
    if not source_video_path:
        return _docs_fallback_path()

    source_dir = os.path.dirname(source_video_path)
    source_name = os.path.basename(source_video_path)
    stem, ext = os.path.splitext(source_name)
    if not ext:
        ext = ".mp4"
    return os.path.join(source_dir, stem + "_detected" + ext)


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


def _read_source_fps(source_video_path):
    source_video_path = str(source_video_path).strip()
    if not source_video_path:
        return 0.0

    cap = cv2.VideoCapture(source_video_path)
    if not cap.isOpened():
        return 0.0
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    cap.release()
    return fps if fps > 0.0 else 0.0


def _select_fps(configured_fps, source_video_path):
    if configured_fps > 0.0:
        return configured_fps
    source_fps = _read_source_fps(source_video_path)
    if source_fps > 0.0:
        return source_fps
    return 30.0


def _read_source_frame_count(source_video_path):
    source_video_path = str(source_video_path).strip()
    if not source_video_path:
        return 0

    cap = cv2.VideoCapture(source_video_path)
    if not cap.isOpened():
        return 0
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    return frame_count if frame_count > 0 else 0


class ImageTopicVideoRecorder:
    def __init__(self):
        self.image_topic = _param("image_topic", "/tag_detections_image")
        source_video_path = _param("source_video_path", "")
        configured_output_path = str(_param("output_path", "")).strip()
        self.output_path = configured_output_path or _derive_output_path(source_video_path)
        self.fps = _select_fps(float(_param("fps", 0.0)), source_video_path)
        self.fourcc = _select_fourcc(self.output_path, _param("fourcc", "auto"))
        self.source_frame_count = _read_source_frame_count(source_video_path)
        self.progress_log_interval = float(_param("progress_log_interval", 1.0))
        self.last_progress_time = 0.0
        self.start_time = time.monotonic()
        self.bridge = CvBridge()
        self.writer = None
        self.frame_count = 0

        output_dir = os.path.dirname(self.output_path)
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        rospy.on_shutdown(self.close)

        rospy.loginfo("[ImageTopicVideoRecorder] image_topic=%s", self.image_topic)
        rospy.loginfo("[ImageTopicVideoRecorder] output_path=%s fps=%.3f fourcc=%s", self.output_path, self.fps, self.fourcc)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "[ImageTopicVideoRecorder] failed to convert image: %s", str(exc))
            return

        if frame is None or frame.size == 0:
            rospy.logwarn_throttle(1.0, "[ImageTopicVideoRecorder] received empty frame")
            return

        if self.writer is None:
            height, width = frame.shape[:2]
            code = cv2.VideoWriter_fourcc(*str(self.fourcc)[:4])
            self.writer = cv2.VideoWriter(self.output_path, code, self.fps, (width, height), True)
            if not self.writer.isOpened():
                rospy.logerr("[ImageTopicVideoRecorder] failed to open output video: %s", self.output_path)
                self.writer = None
                rospy.signal_shutdown("failed to open output video")
                return
            rospy.loginfo("[ImageTopicVideoRecorder] recording %dx%d video", width, height)

        self.writer.write(frame)
        self.frame_count += 1
        now_mono = time.monotonic()
        should_log_progress = (
            self.progress_log_interval <= 0.0
            or (now_mono - self.last_progress_time) >= self.progress_log_interval
        )
        if should_log_progress:
            if self.source_frame_count > 0:
                progress = min(100.0, 100.0 * float(self.frame_count) / float(self.source_frame_count))
                rospy.loginfo(
                    "[ImageTopicVideoRecorder] recording progress=%.1f%% frame=%d/%d",
                    progress,
                    self.frame_count,
                    self.source_frame_count,
                )
            else:
                rospy.loginfo("[ImageTopicVideoRecorder] recording frame=%d", self.frame_count)
            self.last_progress_time = now_mono

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            elapsed = max(1.0e-6, time.monotonic() - self.start_time)
            rospy.loginfo(
                "[ImageTopicVideoRecorder] EXPORT COMPLETE: saved %d frames to %s (elapsed %.1fs)",
                self.frame_count,
                self.output_path,
                elapsed,
            )
        else:
            rospy.logwarn("[ImageTopicVideoRecorder] EXPORT FINISHED WITHOUT OUTPUT: no frames were recorded from %s", self.image_topic)


def main():
    rospy.init_node("image_topic_video_recorder")
    ImageTopicVideoRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
