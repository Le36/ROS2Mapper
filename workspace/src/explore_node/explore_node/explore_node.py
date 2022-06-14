from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

from builtin_interfaces.msg import Duration

from geometry_msgs.msg import PoseStamped 

from std_msgs.msg import String




class ExploreNode(Node):
    def __init__(self) -> None:
        """Create the publisher"""
        super().__init__("explore_node")

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, '/local_costmap/get_costmap')
        self.spin_client = ActionClient(self, Spin, 'spin')
        

        # self.map_occupance_listener = self.create_subscription(
        #     OccupancyGrid,
        #     '/map',
        #     self.map_listener_callback,
        #     1)
        
        self.tf_occupance_listener = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_listener_callback,
            1)

        self.map = []
        self.copyOfMap = []
        self.origin = ()
        self.resolution = 1.0

        # print(self.getLocalCostmap())

        # print("--------")

        # print(self.getGlobalCostmap())

        self.move(-2, -0.5)
        
    def map_listener_callback(self, msg):
        self.map = msg.data
        self.origin = msg.origin
        self.resolution = msg.resolution
        
        self.get_logger().info('I heard: "%s"' % msg)


    def tf_listener_callback(self, msg):



        if msg.transforms[0].header.frame_id == 'odom':
            data = msg.transforms[0].transform.translation

            self.get_logger().info('I heard: "%s"' % data)
    
    def explore(self):

        while(self.check_exploration_status()):

            self.move()

    def check_exploration_status(self):
        self.copyOfMap = self.map
        for x in self.copyOfMap:
            if x == -1:
                return True

        return False


    def move(self, x, y) -> None:
        """Pose to X,Y location"""
        
        goal_msg = NavigateToPose.Goal()
        
        
   
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        #goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose

        self.nav_to_pose_client.wait_for_server()

        return self.nav_to_pose_client.send_goal_async(goal_msg)


        self.nav_to_pose_client.NavigateToPose(goal_pose)



    def getGlobalCostmap(self):
        """Get the global costmap."""
        self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0)
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        """Get the local costmap."""
        self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0)
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    """def spin(self, spin_dist=1.57):
        self.spin_client.wait_for_server(timeout_sec=1.0)
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist

        return self.spin_client.send_goal_async(goal_msg)"""

    def spin(self, spin_dist=6.28318531):

        self.nav_to_pose_client.wait_for_server()
        
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        
        send_goal_future = self.spin_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

def main(args=None) -> None:  
    """Run the node"""
    rclpy.init(args=args)
    explore_node = ExploreNode()
    rclpy.spin(explore_node)
    explore_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
