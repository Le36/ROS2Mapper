import math
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from interfaces.msg import QRCode
from numpy import ndarray
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .math import *

QR_CODE_SIZE = 200 / 1000
CAMERA_MATRIX = np.array(
    [
        [525.44920374, 0.0, 330.24175119],
        [0.0, 526.37302771, 243.26842016],
        [0.0, 0.0, 1.0],
    ]
)
DISTORTION = np.array([[0.25106112, -0.6379611, 0.0069353, 0.01579591, 0.40809116]])
INVERSE_CAMERA_MATRIX = np.linalg.inv(CAMERA_MATRIX)
TF_THRESHOLD = 0.01


class QRCodeReader(Node):
    def __init__(
        self,
        threshold: float = TF_THRESHOLD,
        get_position: Callable[[], Optional[Tuple[Vector3, Quaternion]]] = None,
    ) -> None:
        super().__init__("qr_code_reader")

        self.threshold = threshold
        self.get_position = get_position if get_position else self.get_position_from_tf

        self.found_codes: Dict[int, QRCode] = {}
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        np.set_printoptions(suppress=True, precision=3)
        self.last_update = self.get_clock().now()
        self.position = np.array([[0], [0], [0]])
        self.rotation = np.array([[0], [0], [0]])
        self.rotation_offset = np.array([[-math.pi / 2], [0], [-math.pi / 2]])
        self.camera_position = np.array([[0], [0], [0]])

        self.subscription = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )
        self.publisher = self.create_publisher(QRCode, "/qr_code", 10)
        self.log_publisher = self.create_publisher(String, "/log", 10)

    def undistort_image(self, image: ndarray) -> ndarray:
        """Return undistorted image"""
        h, w = image.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            CAMERA_MATRIX, DISTORTION, (w, h), 1, (w, h)
        )
        undistorted = cv2.undistort(
            image, CAMERA_MATRIX, DISTORTION, None, new_camera_matrix
        )
        return undistorted

    def detect_code(self, image: ndarray) -> Tuple[ndarray, ndarray]:
        """Detect aruco codes from the image and return the corners and ids"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f"DICT_{4}X{4}_{250}")
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(
            gray, arucoDict, parameters=arucoParam
        )
        return ids, bboxs

    def get_vectors(self, points: List) -> List[ndarray]:
        """Get the vectors pointing from the camera to the corners of the aruco code"""
        rotation_matrix = get_rotation_matrix(self.rotation + self.rotation_offset)
        translational_matrix = self.position

        cam = np.array([[0], [0], [0]])
        self.camera_position = translational_matrix + rotation_matrix @ cam
        cam_vec = np.array([[0], [0], [1]])
        self.camera_vector = translational_matrix + rotation_matrix @ cam_vec

        vectors = []
        for point in points:
            p = np.array([[point[0]], [point[1]], [1]])
            pc = INVERSE_CAMERA_MATRIX @ p
            pw = translational_matrix + (rotation_matrix @ pc)
            vector = pw - self.camera_position
            vectors.append(vector)
        return vectors

    def get_position_from_tf(
        self,
    ) -> Optional[Tuple[Vector3, Quaternion]]:
        """Get current robot position from tf_buffer.lookup_transform"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException as exception:
            self.get_logger().info(
                f"Could not transform odom to base_footprint: {exception}"
            )
            return None
        now = self.get_clock().now()
        stamp = transform.header.stamp
        self.last_update = Time(
            seconds=stamp.sec, nanoseconds=stamp.nanosec, clock_type=now.clock_type
        )
        return (transform.transform.translation, transform.transform.rotation)

    def update_position(self) -> None:
        """Get current robot position"""
        pos = self.get_position()
        if not pos:
            return
        position, rotation = pos
        self.position = np.array([[position.x], [position.y], [position.z]])
        self.rotation = quaternion_to_euler(
            np.array([rotation.w, rotation.x, rotation.y, rotation.z])
        )

    def calculate(
        self, points: List[ndarray]
    ) -> Optional[Tuple[ndarray, ndarray, ndarray]]:
        """Calculate the center, normal_vector and rotation of the aruco code"""
        # Change points to format [x, y, z]
        formatted_points = []
        for point in points:
            point.shape = (1, 3)
            formatted_points.append(point[0])

        top_left = formatted_points[0]
        bottom_left = formatted_points[1]
        top_right = formatted_points[2]
        bottom_right = formatted_points[3]

        # Check that the top points are actually higher than the bottom points
        if top_left[2] < bottom_left[2]:
            top_left, bottom_left = bottom_left, top_left
        if top_right[2] < bottom_right[2]:
            top_right, bottom_right = bottom_right, top_right

        # Move vectors until the height diference is equal to the height of the QR code
        if top_left[2] - bottom_left[2] == 0:
            self.get_logger().warn(
                "Top left and bottom left coordinates are at the same height"
            )
            return
        if top_right[2] - bottom_right[2] == 0:
            self.get_logger().warn(
                "Top right and bottom right coordinates are at the same height"
            )
            return
        k = QR_CODE_SIZE / (top_left[2] - bottom_left[2])
        top_left *= k
        bottom_left *= k

        k = QR_CODE_SIZE / (top_right[2] - bottom_right[2])
        top_right *= k
        bottom_right *= k

        # Add the camera position to the points
        new_cam_world = self.camera_position
        new_cam_world.shape = (1, 3)
        new_cam_world = new_cam_world[0]
        top_left += new_cam_world
        bottom_left += new_cam_world
        top_right += new_cam_world
        bottom_right += new_cam_world

        # Calculate the normal vector
        normal_vector = np.cross(top_right - top_left, top_right - bottom_right)
        normal_vector /= np.linalg.norm(normal_vector)

        # Calculate the rotation of the QR code
        rotation_matrix = get_rotation_matrix(self.rotation_offset)
        default_vector = rotation_matrix @ np.array([[0], [0], [1]])
        default_vector.shape = (1, 3)
        default_vector = default_vector[0]
        rotation = quaternion_of_vectors(-normal_vector, default_vector)

        center = (top_left + bottom_right) / 2
        return center, normal_vector, rotation

    def image_callback(self, msg_image: Image) -> None:
        """Find and publish aruco code data and position from the image"""

        image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        image = self.undistort_image(image)
        data_list, corners_list = self.detect_code(image)

        for i, corners in enumerate(corners_list):
            code_id = int(data_list[i][0])
            corners = sorted(corners[0], key=lambda corner: (corner[0], corner[1]))

            self.update_position()
            now = self.get_clock().now()
            diff = (now.nanoseconds - self.last_update.nanoseconds) / 1e9

            if self.threshold >= 0 and abs(diff) > self.threshold:
                self.get_logger().debug(
                    f"Transform too old ({diff} > {self.threshold}), cannot get accurate location for QR code"
                )
                continue

            vectors = self.get_vectors(corners)
            center, normal_vector, rotation = self.calculate(vectors)

            qr_code = QRCode(
                id=code_id,
                center=center,
                normal_vector=normal_vector,
                rotation=rotation,
            )
            if qr_code.id in self.found_codes:
                old_qr_code = self.found_codes[qr_code.id]

                pos_diff = np.linalg.norm(qr_code.center - old_qr_code.center)
                old_angle = quaternion_to_euler(old_qr_code.rotation)[2]
                new_angle = quaternion_to_euler(qr_code.rotation)[2]
                angle_diff = new_angle - old_angle
                while angle_diff < -math.pi:
                    angle_diff += math.tau
                while angle_diff > math.pi:
                    angle_diff -= math.tau

                if pos_diff >= 0.2:
                    self.log_publisher.publish(
                        String(
                            data=f"QR code with id {qr_code.id} has moved over 20cm ({float(pos_diff):.2f}cm), updating position"
                        )
                    )
                elif abs(angle_diff) > math.pi / 9:
                    self.log_publisher.publish(
                        String(
                            data=f"QR code with id {qr_code.id} has turned over 20° ({abs(float(angle_diff/math.pi*180)):.2f}°), updating position"
                        )
                    )
                else:
                    self.get_logger().debug(
                        f"QR code with id {qr_code.id} is in the same position"
                    )
                    continue
            else:
                self.log_publisher.publish(
                    String(data=f"Found a new QR code with id {qr_code.id}")
                )
            self.found_codes[qr_code.id] = qr_code
            self.publisher.publish(qr_code)

    def reset_found_codes(self) -> None:
        """Clear list of found aruco codes"""
        self.found_codes = {}


def main(args=None) -> None:  # pragma: no cover
    """Run the node"""
    rclpy.init(args=args)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    qr_code_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
