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

TIMER_PERIOD = 1.0
SPIN_DIST = 6.28


class ExploreNode(Node):
    """ExploreNode is responsible for creating a navigation plan for the robot
    using nav2's simple api commander. It explores a given area using the
    occupancy grid until there are no more unexplored areas.
    Args:
        nav: a BasicNavigator object which is a
        simple python api for the nav2 package.
    """

    def __init__(self, nav: BasicNavigator) -> None:
        super().__init__("ros2_mapper_explore_node")

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
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.robot_position = [0.0, 0.0, 0.0]
        self.pos_x = -1
        self.pos_y = -1
        self.searching = False
        self.retracing = False
        self.map_set = False
        self.start_time = inf
        self.previous_target = None
        self.spinning = False

        self.retrace_index = 0
        self.retrace_coordinates = []
        self.map = []
        self.map_origin = []
        self.map_matrix = []

    def timer_callback(self):
        if self.map_set:
            if self.searching:
                self.get_logger().info("Calling explore()")
                self.explore()

            if self.retracing:
                self.get_logger().info("Calling retrace()")
                self.retrace()

    def go_to_qr_code(self, qrcode: QRCode) -> None:
        """Navigates to a given QR code
        Args:
            qrcode: QRCode object that has the information about
            its position and rotation
        Returns: None
        """
        self.move(
            qrcode.center[0] + qrcode.normal_vector[0] * 0.5,
            qrcode.center[1] + qrcode.normal_vector[1] * 0.5,
            qrcode.rotation,
        )

    def commander_callback(self, msg: String) -> None:
        """Subscription to the IO_nodes /autonomous_exploration topic,
        which is responsible for handling exploration actions
        Args:
            msg: message that the IO_node sends that handles explorer actions
        Returns: None
        """
        if msg.data == "1":
            self.start_explore()
            self.explore()
        elif msg.data == "2":
            self.cancel_explore()
        else:
            self.get_logger().warn(f"Unknown command code {msg}")

    def map_listener_callback(self, occupancy_grid: OccupancyGrid) -> None:
        """Subscription to the /map topic which is nav2's occupancygrid. The map shows
        where the obstacles are, where there are unoccupied areas and which places are unexplored.
        The method creates an array using this data and saves relevant metadata.
        Args:
            occupancy_grid: an OccupancyGrid object
        Returns: None
        """
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

    def tf_listener_callback(self, msg: TFMessage) -> None:
        """Subscription to the /tf topic which includes information about
        the robots current position. The method updates the robots position whenever
        it hears a new message from /tf that is of the correct type.
        Args:
            msg: topic message that includes all the relevant data concerning
            the robot's position
        Returns: None
        """
        transform: TransformStamped  # Type hint
        for transform in msg.transforms:
            if transform.header.frame_id == "odom":
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.robot_position = [translation.x, translation.y, translation.z]
                self.transform_coordinates_into_grid()

    def make_map(self) -> Optional[Tuple[float, float]]:
        """Transforms the OccupancyGrid map-array into a matrix.
        Args:
            none
        Returns: the next target of exploration
        """
        if not self.map_set:
            self.get_logger().info(str("Map has not been set yet"))
            return

        self.map_matrix = []
        for y in range(self.map_height):
            self.map_matrix.append([])
            for x in range(self.map_width):
                self.map_matrix[y].append(self.map[y * self.map_width + x])

    def find_target(self) -> Optional[Tuple[float, float]]:
        """Finds the next explore target and transforms it into coordinate values.
        Args:
            map: the map_matrix
        Returns: tuple of coordinates of the next target"""
        value = self.breadth_first_search(self.map_matrix, self.pos_x, self.pos_y)
        if not value:
            self.get_logger().info("Finished exploring the whole area")
            return value

        target_x = self.map_origin[0] + value[0] * 0.05
        target_y = self.map_origin[1] + value[1] * 0.05

        return (target_x, target_y)

    def transform_coordinates_into_grid(self) -> None:
        """Convert map coordinate to occupancy grid coordinate
        Args:
            none
        Returns: none"""

        if self.map_origin:
            distance_x = abs(self.robot_position[0] - self.map_origin[0])
            distance_y = abs(self.robot_position[1] - self.map_origin[1])
            self.pos_x = round(distance_x / self.map_resolution)
            self.pos_y = round(distance_y / self.map_resolution)

    def breadth_first_search(
        self, map: List[List[int]], start_x: int, start_y: int
    ) -> Optional[Tuple[int, int]]:
        """Search for closest unexplored area
        Args:
            map: map matrix of occupancy grid
            start_x: starting x position
            start_y: starting y position
        Returns: indeces of the next target or none if everything is found.
        """
        visited = [[False for x in range(len(map[0]))] for y in range(len(map))]
        visited[start_y][start_x] = True
        queue = [(start_x, start_y)]
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

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
                    if abs(start_x - nx) < 10 or abs(start_y - ny) < 10:
                        continue
                    for dir in directions:
                        mx, my = nx + dir[0], ny + dir[1]
                        if 0 <= mx < len(map[0]) and 0 <= my < len(map):
                            continue
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
        """Explore the area by moving to the closest unexplored area. Handles the logic
        of when to call a new target and when to spin
        Args:
            none
        Returns: none
        """
        if not self.nav.isTaskComplete():
            self.get_logger().info("Task is not complete")
            # Get new location if more than 15 seconds have passed from starting the exploration
            # because Nav2 might be stuck
            if 0 < time() - self.start_time < 15:
                return
        else:
            self.get_logger().info("Task **is** complete")

        # if not self.spinning:
        #     self.spinning = True
        #     self.nav.spin(SPIN_DIST)
        #     return

        self.make_map()

        if self.pos_x >= 0 and self.pos_y >= 0:
            target = self.find_target()

        if self.previous_target == target or not target:
            if len(self.retrace_coordinates) == 0:
                return
            self.searching = False
            self.retracing = True
            return

        self.retrace_coordinates.append(target)
        self.previous_target = target

        self.start_time = time()
        # self.spinning = False
        self.move(target[0], target[1])

    def retrace(self):
        """Retrace old explore coordinates"""
        if not self.nav.isTaskComplete():
            return
        if self.retrace_index == len(self.retrace_coordinates):
            self.retrace_index = 0
        target = self.retrace_coordinates[self.retrace_index]
        self.move(target[0], target[1])
        self.retrace_index += 1

    def move(self, x: float, y: float, orientation=None) -> None:
        """Move to pose (x, y) and optionally to a specific orientation"""

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        if orientation is not None:
            goal_pose.pose.orientation.w = orientation[0]
            goal_pose.pose.orientation.x = orientation[1]
            goal_pose.pose.orientation.y = orientation[2]
            goal_pose.pose.orientation.z = orientation[3]
        else:
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
