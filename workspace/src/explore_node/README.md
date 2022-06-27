# Explore node

## Description

Explore node's purpose is to find the next unexplored x,y cordinate and command robot to move there using nav2.

Nav2 is commanded with our version of [nav2 simple commander](../nav2_simple_commander/README.md).

Explore node finds targets using slam toolbox's occupancy grid. Grid is read from topic /map. 

## Requirements

Explore node needs slam and nav2 to funtion properly. 

## Usage

You can run explore node with command ´´´ros2 run explore_node launch´´´

