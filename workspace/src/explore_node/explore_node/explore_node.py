from time import sleep
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

from std_msgs.msg import String


from geometry_msgs.msg import PoseStamped


class ExploreNode(Node):
    def __init__(self, nav: BasicNavigator) -> None:
        """Create the publisher"""
        super().__init__("explore_node")

        self.nav = nav

        self.map_occupance_listener = self.create_subscription(
            OccupancyGrid, "/map", self.map_listener_callback, 1
        )

        self.tf_occupance_listener = self.create_subscription(
            TFMessage, "/tf", self.tf_listener_callback, 1
        )

        self.commander = self.create_subscription(
            String, "/autonomous_exploration", self.commander_callback, 1
        )

        self.robot_positon = [0.0, 0.0, 0.0]
        self.x_index = -1
        self.y_index = -1

        self.searching = False
        self.grid_check = False

        self.map = []
        self.map_origin = []

    def commander_callback(self, msg):
        if msg.data == "1":
            self.searching = True
            self.explore()
        if msg.data == "2":
            self.cancel_explore()

    def map_listener_callback(self, msg):
        self.grid_check = True
        self.map = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            msg.info.origin.position.z,
        ]
        self.map_resolution = msg.info.resolution

        # self.get_logger().info('I heard: "%s"' % [self.map_width, self.map_height, self.map_origin, self.map_resolution])

    def make_map(self):
        if not self.grid_check:
            sleep(5)

        if not self.grid_check:
            print("Can't find occupance grid.")

        map = [[]]

        k = 0
        for i in range(int(self.map_height)):
            map.append([])
            for j in range(int(self.map_width)):
                map[i].append(self.map[k])
                k += 1

        if self.x_index >= 0 and self.y_index >= 0:
            map[self.y_index][self.x_index] = 999
            print("testintg")
            value = self.find_target(map)
            return value

    def find_target(self, map):
        value = self.breadth_first_search(map, self.x_index, self.y_index)
        print("FOUNDFOUNDFOUND")
        if value == 9000:
            print("Kaikki tutkittu")
            return
        map[value[1]][value[0]] = 12345

        for i in map:
            print(i)
        target_x = self.map_origin[0] + value[0] * 0.05
        target_y = self.map_origin[1] + value[1] * 0.05
        print(str(target_x) + " ja " + str(target_y))

        print(self.x_index, self.y_index)
        print(value)
        if map[value[1]][value[0]] == -1:
            print("SUCCESS")
        else:
            print("FAIL")

        return (target_x, target_y)

    def tf_listener_callback(self, msg):

        if msg.transforms[0].header.frame_id == "odom":
            data = msg.transforms[0].transform.translation
            self.robot_positon = [data.x, data.y, data.z]
            self.transform_coordinates_into_grid()

            # self.get_logger().info('I heard: "%s"' % self.robot_positon)

    def transform_coordinates_into_grid(self):

        if self.map_origin:
            distance_x = abs(self.robot_positon[0] - self.map_origin[0])
            distance_y = abs(self.robot_positon[1] - self.map_origin[1])
            self.x_index = round(distance_x / self.map_resolution)
            self.y_index = round(distance_y / self.map_resolution)
            # print(round(self.x_index), round(self.y_index))

    def check_if_inside_map(self, s_x, s_y, i):
        return (
            -1 < s_y + i[0] < self.map_height - 1
            and -1 < s_x + i[1] < self.map_width - 1
        )

    def breadth_first_search(self, map, x, y):
        visited = [[False for i in range(len(map[0]))] for j in range(len(map))]
        visited[y][x] = True
        queue = []
        queue.append((x, y))
        self.searching = True
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        while queue:
            s_x, s_y = queue.pop(0)
            for i in directions:
                if not self.check_if_inside_map(s_x, s_y, i):
                    continue
                if map[s_y + i[0]][s_x + i[1]] == -1:
                    if abs(x - s_x + i[0]) < 10 or abs(y - s_y + i[1]) < 10:
                        continue
                    print(s_x + i[0])
                    print(s_y + i[1])
                    return [s_x + i[1], s_y + i[0]]
                elif (
                    map[s_y + i[0]][s_x + i[1]] == 0
                    and visited[s_y + i[0]][s_x + i[1]] == False
                ):
                    visited[s_y + i[0]][s_x + i[1]] = True
                    queue.append((s_x + i[1], s_y + i[0]))

        return 9000

    def cancel_explore(self):
        self.searching = False
        self.nav.cancelTask()

    def explore(self):

        while self.searching:
            if not self.nav.isTaskComplete():
                print("testi")
                continue

            target = self.make_map()
            print(target)
            if target == None:
                print("testiä2345")
                continue
            print("testiä")
            self.move(target[0], target[1])
            # self.nav.spin()

    def move(self, x, y) -> None:
        """Pose to X,Y location"""

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        # goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.nav.goToPose(goal_pose)

    # def spin(self, spin_dist=6.28318531):

    #     self.nav_to_pose_client.wait_for_server()
    #     self.spin_client.wait_for_server()

    #     goal_msg = Spin.Goal()
    #     goal_msg.target_yaw = spin_dist

    #     send_goal_future = self.spin_client.send_goal_async(goal_msg)


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
