from math import inf
from time import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Vector3
from interfaces.msg import QRCode
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


class ExploreNode(Node):
    def __init__(self, nav: BasicNavigator) -> None:
        super().__init__("explore_node")

        self.nav = nav

        self.map_occupancy_listener = self.create_subscription(
            OccupancyGrid, "/map", self.map_listener_callback, 1
        )
        self.tf_listener = self.create_subscription(
            TFMessage, "/tf", self.tf_listener_callback, 1
        )
        self.commander_subscription = self.create_subscription(
            String, "/autonomous_exploration", self.commander_callback, 1
        )
        self.go_to_qr_code_subscription = self.create_subscription(
            QRCode, "/qr_navigator", self.go_to_qr_code, 1
        )

        self.robot_position = [0.0, 0.0, 0.0]
        self.pos_x = -1
        self.pos_y = -1
        self.initial_pose = None
        self.searching = False
        self.retracing = False
        self.map_set = False
        self.start_time = inf
        self.previous_target = None

        self.retrace_index = 0
        self.retrace_coordinates = []
        self.map = []
        self.map_origin = []

    def go_to_qr_code(self, qrcode: QRCode) -> None:
        """Navigates to a given QR code"""
        self.move(
            qrcode.center[0] + qrcode.normal_vector[0] * 0.2,
            qrcode.center[1] + qrcode.normal_vector[1] * 0.2,
        )

    def commander_callback(self, msg: String) -> None:
        """Handle commands from the IO node"""
        if msg.data == "1":
            self.searching = True
            self.explore()
        elif msg.data == "2":
            self.cancel_explore()
        else:
            self.get_logger().warn(f"Unknown command code {msg}")

    def map_listener_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """Listen to occupance grid. Update map and run exploration/retracing"""
        if not self.map_set:
            self.get_logger().info("Ready to explore!")
        if not occupancy_grid.header.frame_id == "map":
            return
        self.map_set = True
        self.map = occupancy_grid.data
        self.map_width = occupancy_grid.info.width
        self.map_height = occupancy_grid.info.height
        self.map_origin = [
            occupancy_grid.info.origin.position.x,
            occupancy_grid.info.origin.position.y,
            occupancy_grid.info.origin.position.z,
        ]
        self.map_resolution = occupancy_grid.info.resolution

        self.get_logger().info(str(occupancy_grid))

        if self.initial_pose is not None:
            if self.searching:
                self.explore()

            if self.retracing:
                self.retrace()

    def tf_listener_callback(self, msg: TFMessage) -> None:
        """Update robot position"""
        transform: TransformStamped  # Type hint
        for transform in msg.transforms:
            if transform.header.frame_id == "odom":
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.robot_position = [translation.x, translation.y, translation.z]
                if self.initial_pose is None:
                    self.set_initial_pose(translation, rotation)
                self.transform_coordinates_into_grid()

    def set_initial_pose(self, translation: Vector3, rotation: Quaternion) -> None:
        """Set initial pose"""
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = "map"
        self.initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = translation.x
        self.initial_pose.pose.position.y = translation.y
        self.initial_pose.pose.orientation.z = rotation.z
        self.initial_pose.pose.orientation.w = rotation.w

        # Don't set initial pose, because it breaks in Gazebo
        # self.nav.setInitialPose(self.initial_pose)

    def make_map(self) -> Optional[Tuple[float, float]]:
        """Turn occupance grid to matrix. Returns next explore target."""
        if not self.map_set:
            self.get_logger().info(str("Map has not been set yet"))
            return

        map = []
        for y in range(self.map_height):
            map.append([])
            for x in range(self.map_width):
                map[y].append(self.map[y * self.map_width + x])

        for x in map:
            self.get_logger().info(str(x))

        if self.pos_x >= 0 and self.pos_y >= 0:
            value = self.find_target(map)
            return value

    def find_target(self, map: List[List[int]]) -> Optional[Tuple[float, float]]:
        """Find next explore target. Returns nav2 x,y cordinates."""
        value = self.breadth_first_search(map, self.pos_x, self.pos_y)
        if not value:
            self.get_logger().info("Finished exploring the whole area")
            return value

        target_x = self.map_origin[0] + value[0] * 0.05
        target_y = self.map_origin[1] + value[1] * 0.05

        return (target_x, target_y)

    def transform_coordinates_into_grid(self) -> None:
        """Convert map coordinate to occupancy grid coordinate"""
        if self.map_origin:
            distance_x = abs(self.robot_position[0] - self.map_origin[0])
            distance_y = abs(self.robot_position[1] - self.map_origin[1])
            self.pos_x = round(distance_x / self.map_resolution)
            self.pos_y = round(distance_y / self.map_resolution)

    def breadth_first_search(
        self, map: List[List[int]], start_x: int, start_y: int
    ) -> Optional[Tuple[int, int]]:
        """Search for closest unexplored area"""
        visited = [[False for x in range(len(map))] for y in range(len(map[0]))]
        self.get_logger().info(
            str(len(visited))
            + " ja "
            + str(len(visited[0]))
            + " ja x "
            + str(start_x)
            + " ja y "
            + str(start_y)
        )
        visited[start_y][start_x] = True
        queue = [(start_x, start_y)]
        self.searching = True
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        self.get_logger().info(
            "Width " + str(self.map_width) + " Height " + str(self.map_height)
        )
        self.get_logger().info(str(len(map)) + " " + str(len(map[0])))

        while queue:
            x, y = queue.pop(0)
            for dir in directions:
                nx, ny = x + dir[0], y + dir[1]
                if not (0 <= nx < self.map_width and 0 <= ny < self.map_height):
                    continue
                elif map[ny][nx] == -1 or (
                    map[ny][nx] < 50
                    and (
                        ny == 0
                        or ny == self.map_height - 1
                        or nx == 0
                        or nx == self.map_width - 1
                    )
                ):
                    # if abs(start_x - nx) < 10 or abs(start_y - ny) < 10:
                    #    continue
                    self.get_logger().info("Koordinaatin arvo: " + str(map[ny][nx]))
                    return (nx, ny)
                elif map[ny][nx] < 70 and not visited[ny][nx]:
                    visited[ny][nx] = True
                    queue.append((nx, ny))

        return None

    def cancel_explore(self) -> None:
        """Cancel exploration"""
        self.searching = False
        self.retracing = False
        self.nav.cancelTask()

    def start_explore(self) -> None:
        """Start exploration"""
        self.searching = True

    def explore(self) -> None:
        """Explore the area by moving to the closest unexplored area"""
        if not self.nav.isTaskComplete():
            self.get_logger().info("Task is not complete")
            # Get new location if more than 15 seconds have passed from starting the exploration
            # because Nav2 might be stuck
            if 0 < time() - self.start_time < 15:
                return
        else:
            self.get_logger().info("Task **is** complete")

        target = self.make_map()

        if not target:
            target = self.make_map()

        if self.previous_target == target or not target:
            if len(self.retrace_coordinates) == 0:
                return
            self.searching = False
            self.retracing = True
            return

        self.retrace_coordinates.append(target)
        self.previous_target = target

        self.start_time = time()
        self.move(target[0], target[1])

    def retrace(self):
        """Retrace old explore coordinates"""
        if not self.nav.isTaskComplete():
            return
        if self.retrace_index == len(self.retrace_coordinates):
            self.retrace_index = 0
        target = self.retrace_coordinates[self.retrace_index]
        self.move_and_spin(target[0], target[1])
        self.retrace_index += 1

        self.make_map()

    def move(self, x: float, y: float) -> None:
        """Move to pose (x, y)"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.nav.goToPose(goal_pose)

    def move_and_spin(self, x: float, y: float) -> None:
        """Move to pose (x, y) and spin at the end"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.nav.moveAndSpin(goal_pose)


def main(args=None) -> None:
    """Run the node"""
    rclpy.init(args=args)
    nav = BasicNavigator()
    explore_node = ExploreNode(nav)
    rclpy.spin(explore_node)
    explore_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
