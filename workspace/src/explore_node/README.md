# Explore node

## Description

Explore node's purpose is to find the next unexplored x,y cordinate and command robot to move there using nav2.

Nav2 is commanded with our version of [nav2 simple commander](../nav2_simple_commander/README.md).

Explore node subscribes to slam toolbox's `/map` and `/tf` topics to get the occupancy grid and the robots position on the map respectively.  

## Requirements

Explore node needs slam and nav2 to funtion properly. 

## Usage
```
ros2 run explore_node launch
```
