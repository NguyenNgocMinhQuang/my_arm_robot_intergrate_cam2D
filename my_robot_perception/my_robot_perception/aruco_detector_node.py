#!/usr/bin/env python3
"""
aruco_detector_node.py

Node Perception: đọc ảnh từ webcam 2D (qua topic image_raw + camera_info do
một camera driver khác publish, ví dụ usb_cam / v4l2_camera), phát hiện
ArUco marker bằng OpenCV, ước lượng pose 3D bằng solvePnP, rồi:
  1. Publish geometry_msgs/PoseStamped lên topic /detected_object_pose
  2. Broadcast TF frame "aruco_marker_frame" so với frame của camera

LƯU Ý QUAN TRỌNG VỀ HỆ QUY CHIẾU:
- OpenCV solvePnP trả pose trong "optical frame" của camera (X phải, Y xuống,
  Z hướng ra trước ống kính) - đây CŨNG chính là quy ước "camera_optical_frame"
  chuẩn của ROS (REP-103). Vì vậy header.frame_id của PoseStamped ở đây PHẢI
  là camera_optical_frame_id (tham số camera_optical_frame_id bên dưới),
  KHÔNG phải camera_link (camera_link theo quy ước ROS thường là X-forward,
  Z-up, khác với optical frame).
- Bạn cần publish thêm 1 static TF camera_link -> camera_optical_frame_id
  (xoay chuẩn) trong launch file, cùng với static TF base_link -> camera_link
  (hiệu chuẩn ngoại vi thực tế của bạn). Khi đó chuỗi TF
  base_link -> camera_link -> camera_optical_frame -> aruco_marker_frame
  sẽ tự động đúng và State Machine chỉ cần tra cứu TF là ra tọa độ vật
  trong base_link.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(rot_matrix: np.ndarray):
    """Chuyển ma trận xoay 3x3 -> quaternion (x, y, z, w)."""
    m = rot_matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        # ---------------- Tham số cấu hình ----------------
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_optical_frame_id", "camera_optical_frame")
        self.declare_parameter("aruco_marker_frame_id", "aruco_marker_frame")
        self.declare_parameter("target_marker_id", 0)  # -1 = chấp nhận mọi id
        self.declare_parameter("marker_length", 0.04)  # 4cm, theo đề bài
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("publish_debug_image", True)

        self.image_topic_ = self.get_parameter("image_topic").value
        self.camera_info_topic_ = self.get_parameter("camera_info_topic").value
        self.camera_optical_frame_id_ = self.get_parameter("camera_optical_frame_id").value
        self.aruco_marker_frame_id_ = self.get_parameter("aruco_marker_frame_id").value
        self.target_marker_id_ = self.get_parameter("target_marker_id").value
        self.marker_length_ = self.get_parameter("marker_length").value
        dict_name = self.get_parameter("aruco_dictionary").value
        self.publish_debug_image_ = self.get_parameter("publish_debug_image").value

        # ---------------- OpenCV / ArUco setup ----------------
        self.bridge_ = CvBridge()
        aruco_dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict_ = cv2.aruco.getPredefinedDictionary(aruco_dict_id)

        # Hỗ trợ cả API mới (ArucoDetector, OpenCV >= 4.7) lẫn API cũ hơn
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector_params_ = cv2.aruco.DetectorParameters()
            self.aruco_detector_ = cv2.aruco.ArucoDetector(self.aruco_dict_, self.detector_params_)
            self.use_new_api_ = True
        else:
            self.detector_params_ = cv2.aruco.DetectorParameters_create()
            self.use_new_api_ = False

        # Điểm 3D của 4 góc marker trong hệ tọa độ tâm marker (đơn vị: mét)
        half = self.marker_length_ / 2.0
        self.object_points_ = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        self.camera_matrix_ = None
        self.dist_coeffs_ = None

        # ---------------- ROS interfaces ----------------
        self.image_sub_ = self.create_subscription(
            Image, self.image_topic_, self.image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, self.camera_info_topic_, self.camera_info_callback, 10
        )
        self.pose_pub_ = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
        if self.publish_debug_image_:
            self.debug_image_pub_ = self.create_publisher(Image, "/aruco_detector/debug_image", 10)

        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.get_logger().info(
            f"Aruco detector started. Waiting for image on '{self.image_topic_}' "
            f"and camera_info on '{self.camera_info_topic_}'."
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix_ = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs_ = np.array(msg.d, dtype=np.float64)

    def image_callback(self, msg: Image):
        if self.camera_matrix_ is None:
            # Chưa có thông tin hiệu chuẩn camera thì chưa thể solvePnP chính xác
            return

        frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_api_:
            corners, ids, _ = self.aruco_detector_.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict_, parameters=self.detector_params_
            )

        if ids is None:
            if self.publish_debug_image_:
                self.debug_image_pub_.publish(self.bridge_.cv2_to_imgmsg(frame, encoding="bgr8"))
            return

        for i, marker_id in enumerate(ids.flatten()):
            if self.target_marker_id_ != -1 and marker_id != self.target_marker_id_:
                continue

            image_points = corners[i][0].astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                self.object_points_,
                image_points,
                self.camera_matrix_,
                self.dist_coeffs_,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not success:
                continue

            rot_matrix, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = rotation_matrix_to_quaternion(rot_matrix)

            now = self.get_clock().now().to_msg()

            # ---- Publish PoseStamped ----
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.camera_optical_frame_id_
            pose_msg.pose.position.x = float(tvec[0][0])
            pose_msg.pose.position.y = float(tvec[1][0])
            pose_msg.pose.position.z = float(tvec[2][0])
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self.pose_pub_.publish(pose_msg)

            # ---- Broadcast TF ----
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.camera_optical_frame_id_
            t.child_frame_id = self.aruco_marker_frame_id_
            t.transform.translation.x = pose_msg.pose.position.x
            t.transform.translation.y = pose_msg.pose.position.y
            t.transform.translation.z = pose_msg.pose.position.z
            t.transform.rotation = pose_msg.pose.orientation
            self.tf_broadcaster_.sendTransform(t)

            if self.publish_debug_image_:
                cv2.drawFrameAxes(
                    frame, self.camera_matrix_, self.dist_coeffs_, rvec, tvec, self.marker_length_ * 0.5
                )

            # Chỉ xử lý 1 marker mục tiêu mỗi khung hình là đủ cho pick&place
            break

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if self.publish_debug_image_:
            self.debug_image_pub_.publish(self.bridge_.cv2_to_imgmsg(frame, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
