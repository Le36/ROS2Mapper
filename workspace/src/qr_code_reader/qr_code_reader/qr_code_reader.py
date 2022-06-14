import math
from typing import List, Optional, Tuple

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from mpl_toolkits import mplot3d
from numpy import ndarray
from pyzbar import pyzbar
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .math import *

QR_CODE_SIZE = 200 / 1000
WIDTH, HEIGHT = 1024, 768
CAMERA_MATRIX = np.array([[612.48, 0, WIDTH / 2], [0, 612.29, HEIGHT / 2], [0, 0, 1]])

INVERSE_CAMERA_MATRIX = np.linalg.inv(CAMERA_MATRIX)

DRAW = True
DRAW_ROBOT = False
PLOT_AXIS_SIZE = 5000 / 1000


class QRCodeReader(Node):
    def __init__(self) -> None:
        """Create the subscriber and the publisher"""
        super().__init__("qr_code_reader")

        self.found_codes = []
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        np.set_printoptions(suppress=True, precision=3)
        self.position = np.array([[0], [0], [0]])
        self.rotation = np.array([[0], [0], [0]])
        self.rotation_offset = np.array([[-math.pi / 2], [0], [-math.pi / 2]])
        self.camera_position = np.array([[0], [0], [0]])

        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.publisher = self.create_publisher(String, "/add_data", 10)

        if DRAW:
            matplotlib.interactive(True)
            self.ax = plt.axes(projection="3d")
            self.ax.set_xlim3d(-PLOT_AXIS_SIZE, PLOT_AXIS_SIZE)
            self.ax.set_ylim3d(-PLOT_AXIS_SIZE, PLOT_AXIS_SIZE)
            self.ax.set_zlim3d(-PLOT_AXIS_SIZE, PLOT_AXIS_SIZE)
            self.draw_axes()
            plt.show()
            plt.draw()
            plt.pause(0.01)

    # def undistort_image(self, image: ndarray) -> ndarray:
    #     h, w = image.shape[:2]
    #     new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    #         CAMERA_MATRIX, DISTORTION, (w, h), 1, (w, h)
    #     )
    #     undistorted = cv2.undistort(
    #         image, CAMERA_MATRIX, DISTORTION, None, new_camera_matrix
    #     )
    #     return undistorted

    def get_vectors(self, points: List) -> List[ndarray]:
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

    def update_position(self) -> bool:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time()
            )
        except TransformException as exception:
            self.get_logger().info(
                f"Could not transform odom to base_footprint: {exception}"
            )
            return
        now = self.get_clock().now()
        stamp = transform.header.stamp
        transform_time = Time(
            seconds=stamp.sec, nanoseconds=stamp.nanosec, clock_type=ClockType.ROS_TIME
        )
        diff = (now.nanoseconds - transform_time.nanoseconds) / 1e9
        # self.get_logger().info(f"Transform: {transform_time}")
        # self.get_logger().info(f"Now:       {now}")
        # self.get_logger().info(f"Diff:      {diff}")

        # if abs(diff) > 0.1:
        if abs(diff) > 1:
            return False

        position = transform.transform.translation
        rotation = transform.transform.rotation
        self.position = np.array([[position.x], [position.y], [position.z]])
        self.rotation = quaternion_to_euler(
            np.array([rotation.w, rotation.x, rotation.y, rotation.z])
        )
        return True

    def draw_line(self, point1, point2, color="blue") -> None:
        self.ax.plot(
            [point1[0], point2[0]],
            [point1[1], point2[1]],
            [point1[2], point2[2]],
            color=color,
        )

    def draw_axes(self) -> None:
        self.draw_line(
            np.array([-PLOT_AXIS_SIZE, 0, 0]),
            np.array([PLOT_AXIS_SIZE, 0, 0]),
            "black",
        )
        self.draw_line(
            np.array([0, -PLOT_AXIS_SIZE, 0]),
            np.array([0, PLOT_AXIS_SIZE, 0]),
            "black",
        )
        self.draw_line(
            np.array([0, 0, -PLOT_AXIS_SIZE]),
            np.array([0, 0, PLOT_AXIS_SIZE]),
            "black",
        )

    def draw_robot(self) -> None:
        self.get_vectors([])  # Update camera position and camera vector
        new_cam_world = self.camera_position
        new_cam_world.shape = (1, 3)
        new_cam_world = new_cam_world[0]

        self.draw_line(new_cam_world, self.camera_vector)
        self.ax.scatter(
            new_cam_world[0], new_cam_world[1], new_cam_world[2], color="g", s=100
        )

    def calculate(
        self, points: List[ndarray]
    ) -> Optional[Tuple[ndarray, ndarray, ndarray]]:
        # Change points to format [x, y, z]
        formatted_points = []
        for point in points:
            point.shape = (1, 3)
            formatted_points.append(point[0])

        top_left = formatted_points[0]
        bottom_left = formatted_points[1]
        top_right = formatted_points[2]
        bottom_right = formatted_points[3]

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
        rotation = quaternion_of_vectors(normal_vector, default_vector)

        center = (top_left + bottom_right) / 2
        if DRAW:
            self.draw_line(top_right, top_left)
            self.draw_line(top_right, bottom_right)
            self.draw_line(bottom_left, top_left)
            self.draw_line(bottom_left, bottom_right)

            self.draw_line(
                center, center + normal_vector * (PLOT_AXIS_SIZE * 2) / 10, "red"
            )

        return center, normal_vector, rotation

    def image_callback(self, msg_image: Image) -> None:
        """Find and publish QR code data"""
        if DRAW:
            self.draw_axes()
            if DRAW_ROBOT and self.update_position():
                self.draw_robot()

        image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        # image = self.undistort_image(image)
        codes = pyzbar.decode(image)
        for code in codes:
            qr_code_points = list(code.polygon)
            qr_code_points.sort()

            if not self.update_position():
                continue
            vectors = self.get_vectors(qr_code_points)
            center, normal_vector, rotation = self.calculate(vectors)

            data = code.data.decode()
            if data in self.found_codes:
                continue
            self.found_codes.append(data)
            self.get_logger().info(f"Found new a QR code with data '{data}'")
            self.publisher.publish(String(data=data))

        if DRAW:
            plt.draw()
            plt.pause(0.01)

    def reset_found_codes(self) -> None:
        self.found_codes = []


def main(args=None) -> None:  # pragma: no cover
    """Run the node"""
    rclpy.init(args=args)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    qr_code_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
