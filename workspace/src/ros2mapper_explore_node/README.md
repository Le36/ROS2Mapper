# Explore node

## Description
This node finds the nearest unexplored (x, y) coordinates and commands the robot to move there using Nav2. This node also handles the navigation to found QR codes.

This node commands Nav2 with our version of [nav2 simple commander](../nav2_simple_commander) and this node is commanded by the [IO node](../io_node). This node subscribes to slam toolbox's `/map` and `/tf` topics to get the occupancy grid and the robot's position on the map respectively.  

## Dependencies
- rclpy
- geometry_msgs
- interfaces
- nav2_simple_commander
- nav_msgs
- std_msgs
- tf2_msgs

### Topics
| Publish/Subscribe | Topic                     | Type                       | Description                                                                                                    |
| ----------------- | ------------------------- | -------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Subscribe         | `/tf`                     | tf2_msgs/msg/TFMessage     | Listens to the robot's position                                                                                |
| Subscribe         | `/map`                    | nav_msgs/msg/OccupancyGrid | Listens to the OccupancyGrid-map data which shows obstacles and unexplored areas                               |
| Subscribe         | `/autonomous_exploration` | std_msgs/msg/String        | Listens to the commands to start ("1") the autonomous exploration and to stop ("2") the autonomous exploration |
| Subscribe         | `/qr_navigator`           | interfaces/msg/QRCode      | Listens for a QR code to navigate to                                                                           |

## Usage
```
ros2 run ros2mapper_explore_node launch
```
