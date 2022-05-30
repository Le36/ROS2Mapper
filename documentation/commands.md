# Commands.md

## Table of contents
- [Table of contents](#table-of-contents)
- [Commands](#commands)
    - [Launching the Turtlebot3 worlds](#launching-the-turtlebot3-worlds)
    - [Controlling the Turtlebot3](#controlling-the-turtlebot3)
    - [Launching the Cartographer](#launching-the-cartographer)

## Commands
### Launching the Turtlebot3 worlds
- turtlebot3_world
    ```
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
- turtlebot3_house
    ```
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
    ```

### Controlling the Turtlebot3
```
ros2 run turtlebot3_teleop teleop_keyboard
```

### Launching the Cartographer
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```