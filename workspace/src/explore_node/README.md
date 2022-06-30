# Explore node

## Description

Explore node's purpose is to find the next unexplored x,y cordinate and command robot to move there using nav2 and handle the navigation to found QR-codes.

Nav2 is commanded with our version of [nav2 simple commander](../nav2_simple_commander).

Explore_node's actions are commanded by [IO node](../io_node).

Explore node subscribes to slam toolbox's `/map` and `/tf` topics to get the occupancy grid and the robots position on the map respectively.  

### Topics

| Package | Subscribes to topic     | Description                         |
|---------| ----------------------- | ----------------------------------- |
| SLAM    | /tf                     | Data of robots current position     |
| nav2    | /map                    | OccupancyGrid-map data which shows obstacles and unexplored places    |
| IO node | /autonomous_exploration | Gives the commands for handling the actions of explore_node       |
| IO node | /qr_navigator           | Gives the ids and data of the QR-codes where to navigate |


## Dependencies

- rclpy
- geometry_msgs
- interfaces
- nav2_simple_commander
- nav_msgs
- std_msgs
- tf2_msgs

## Usage
```
ros2 run explore_node launch
```
