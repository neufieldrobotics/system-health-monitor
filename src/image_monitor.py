"""
This is a generic node that will check the health of images being published.
It can check both
- monocular images
- stereo images

It will check the following:
- if the images are being published at the correct rate
- if there are signs of image corruption
- image size and resolution
- image blur or other quality metrics
"""
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Dict, Optional
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class ImageMonitor:
    def __init__(self, node: Node, image_topics: List[str], camera_info_topics: List[str]):
        self.node = node
        self.bridge = CvBridge()
        self.image_topics = image_topics
        self.camera_info_topics = camera_info_topics

        self.last_timestamps: Dict[str, float] = {}
        self.frame_rates: Dict[str, float] = {}
        self.blur_scores: Dict[str, float] = {}
        self.frame_rate_avg: Dict[str, float] = {}
        self.blur_score_avg: Dict[str, float] = {}

        self.ema_alpha = 0.2

        self.image_subs = [
            node.create_subscription(Image, topic, self.make_image_callback(topic), 10)
            for topic in self.image_topics
        ]

        self.camera_info_data: Dict[str, CameraInfo] = {}
        self.camera_info_subs = [
            node.create_subscription(CameraInfo, topic, self.make_camera_info_callback(topic), 10)
            for topic in self.camera_info_topics
        ]

    def make_image_callback(self, topic: str):
        def callback(msg: Image):
            self.handle_image_message(msg, topic)
        return callback

    def make_camera_info_callback(self, topic: str):
        def callback(msg: CameraInfo):
            self.camera_info_data[topic] = msg
        return callback

    def handle_image_message(self, msg: Image, topic: str, encoding: str = 'bgr8'):
        now = self._to_sec(msg.header.stamp)
        last = self.last_timestamps.get(topic)
        if last:
            dt = now - last
            if dt > 0:
                freq = 1.0 / dt
                self.frame_rates[topic] = freq
                self._update_moving_average(self.frame_rate_avg, topic, freq)
        self.last_timestamps[topic] = now

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
            blur = self.check_blur(img)
            self.blur_scores[topic] = blur
            self._update_moving_average(self.blur_score_avg, topic, blur)
        except Exception as e:
            self.node.get_logger().warn(f"Image decode failed for topic {topic}: {e}")

    def check_blur(self, img: np.ndarray) -> float:
        if img is None or img.size == 0:
            return 0.0
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def _update_moving_average(self, store: Dict[str, float], key: str, new_value: float):
        old_value = store.get(key, new_value)
        store[key] = self.ema_alpha * new_value + (1 - self.ema_alpha) * old_value

    def _to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def get_status(self) -> List[DiagnosticStatus]:
        status_list = []
        for topic in self.image_topics:
            status = DiagnosticStatus()
            status.name = f"ImageMonitor: {topic}"
            status.level = DiagnosticStatus.OK
            status.message = "OK"

            blur = self.blur_scores.get(topic, -1)
            freq = self.frame_rates.get(topic, -1)
            blur_avg = self.blur_score_avg.get(topic, -1)
            freq_avg = self.frame_rate_avg.get(topic, -1)

            status.values.extend([
                KeyValue("blur_instant", f"{blur:.2f}"),
                KeyValue("blur_avg", f"{blur_avg:.2f}"),
                KeyValue("frequency_instant", f"{freq:.2f} Hz" if freq > 0 else "N/A"),
                KeyValue("frequency_avg", f"{freq_avg:.2f} Hz" if freq_avg > 0 else "N/A"),
            ])

            if blur < 20:
                status.level = DiagnosticStatus.WARN
                status.message = "Blurry image"

            if freq > 0 and freq < 5:
                status.level = DiagnosticStatus.WARN
                status.message += " + Low frame rate" if status.message != "OK" else "Low frame rate"

            if freq_avg > 0 and freq_avg < 5:
                status.level = DiagnosticStatus.WARN
                status.message += " + Low avg frame rate" if status.message != "OK" else "Low avg frame rate"

            status_list.append(status)

        return status_list


class StereoImageMonitor(ImageMonitor):
    def __init__(self, node, left_topic: str, right_topic: str, camera_info_topics: Optional[List[str]] = None, sync_slop: float = 0.02):
        super().__init__(node, [left_topic, right_topic], camera_info_topics or [])

        self.left_topic = left_topic
        self.right_topic = right_topic
        self.sync_slop = sync_slop

        self.left_sub = Subscriber(node, Image, left_topic)
        self.right_sub = Subscriber(node, Image, right_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub], queue_size=10, slop=sync_slop
        )
        self.sync.registerCallback(self.stereo_callback)

        self.latest_results = DiagnosticStatus()
        self.latest_results.name = "StereoImageMonitor"

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
        right_img = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

        self.handle_image_message(left_msg, self.left_topic)
        self.handle_image_message(right_msg, self.right_topic)

        status = DiagnosticStatus()
        status.name = "StereoImageMonitor"
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        t_left = self._to_sec(left_msg.header.stamp)
        t_right = self._to_sec(right_msg.header.stamp)
        time_diff = abs(t_left - t_right)
        status.values.append(KeyValue("timestamp_diff", f"{time_diff:.4f} s"))
        if time_diff > self.sync_slop:
            status.level = DiagnosticStatus.WARN
            status.message = "Timestamp mismatch"

        left_brightness = float(np.mean(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)))
        right_brightness = float(np.mean(cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)))
        brightness_diff = abs(left_brightness - right_brightness)

        status.values.extend([
            KeyValue("left_brightness", f"{left_brightness:.2f}"),
            KeyValue("right_brightness", f"{right_brightness:.2f}"),
            KeyValue("brightness_diff", f"{brightness_diff:.2f}"),
        ])

        if brightness_diff > 20:
            status.level = max(status.level, DiagnosticStatus.WARN)
            status.message += " + Brightness mismatch" if status.message != "OK" else "Brightness mismatch"

        if left_img.shape != right_img.shape:
            status.level = max(status.level, DiagnosticStatus.ERROR)
            status.message += " + Resolution mismatch" if status.message != "OK" else "Resolution mismatch"

        self.latest_results = status

    def get_status(self) -> List[DiagnosticStatus]:
        return [self.latest_results] + super().get_status()
