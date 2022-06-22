# Table of contents

- [VS Code](#vs-code)
    - [Configuring markdown autoformat in VS Code](#configuring-markdown-autoformat-in-vs-code)
- [Gazebo](#gazebo)
    - [SLAM in the Gazebo simulation](#slam-in-the-gazebo-simulation)
    - [Autonomous exploration in the Gazebo simulator](#autonomous-exploration-in-the-gazebo-simulator)
- [Physical robot](#physical-robot)
    - [Nav2 and SLAM on the physical robot](#nav2-and-slam-on-the-physical-robot)
    - [Getting the output from the Raspberry Pi camera](#getting-the-output-from-the-raspberry-pi-camera)
    - [Autonomous exploration on the physical robot](#autonomous-exploration-on-the-physical-robot)
- [Project in Gazebo](#project-in-gazebo)
    - [Running the project](#running-the-project)
    - [Running the linter](#running-the-linter)
    - [Running the tests](#running-the-tests)
    - [Generating the coverage report](#generating-the-coverage-report)
- [Project on the physical robot](#project-on-the-physical-robot)
    - [Running the project](#running-the-project-1)

# Tutorials

## VS Code

### Configuring markdown autoformat in VS Code

1. Install the Markdown All in One extension
    1. Press `Ctrl+Shift+X` to open the extensions tab
    2. Search for `Markdown All in One` and install it
2. Configure VS Code settings
    1. Press `Ctrl+Comma` to open the settings
    2. Search for `format on save` and check the box
    3. Search for `indentation size` and select the option `inherit`
    4. Search for `toc.levels` and change the value to `2..6`

## Gazebo

### SLAM in the Gazebo simulation

([link to the tutorial](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/))

1. Launch the simulation world
    ```
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```
2. Run SLAM node
    ```
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
    ```
3. Control the robot
    ```
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

### Autonomous exploration in the Gazebo simulator

Run Gazebo, Nav2, and the exploration

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py
ros2 launch explore_lite explore.launch.py
```

## Physical robot

### Nav2 and SLAM on the physical robot

([link to the tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html))

1. [Raspi] Run turtlebot3_bringup
    ```
    ros2 launch turtlebot3_bringup robot.launch.py
    ```
2. [Remote] Run Nav2 and SLAM
    ```
    ros2 launch turtlebot3_navigation2 navigation2.launch.py
    ros2 launch slam_toolbox online_async_launch.py
    ```
3. [Remote] Control the robot
    ```
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

### Getting the output from the Raspberry Pi camera

1. [Raspi] Run the camera node (in a new terminal)
    ```
    cd ~/ros2_ws
    . install/local_setup.bash
    ros2 run v4l2_camera v4l2_camera_node
    ```
   The last command gives errors because apparently, the Raspberry Pi camera doesn't have all the configuration settings
   a regular camera would have. The errors are probably safe to ignore.
2. [Remote] View the output of the camera
    1. Run the following command
       ```
       ros2 run rqt_image_view rqt_image_view
       ```
    2. When rqt has opened, press "Refresh topics"
    3. Select "/image_raw/compressed" from the dropdown menu

### Autonomous exploration on the physical robot

1. [Raspi] Run turtlebot3_bringup
    ```
    ros2 launch turtlebot3_bringup robot.launch.py
    ```
2. [Remote] Run Nav2, SLAM, and the exploration
    ```
    ros2 launch turtlebot3_navigation2 navigation2.launch.py
    ros2 launch slam_toolbox online_async_launch.py
    ros2 launch explore_lite explore.launch.py
    ```

## Project in Gazebo

**Note: The following commands must be run in the workspace folder of the repository**  
**Note: Remember to source venv and install/setup.bash before running any of the commands**

### Running the project

```
./run.sh
```

### Running the linter

```
./lint.sh
```

### Running the tests

```
pytest src
```

### Generating the coverage report

```
coverage run --branch -m pytest src && coverage html
```

## Project on the physical robot

### Running the project

1. [Remote] Find out the IP of the Raspberry Pi
2. [Remote] Run the launch script
    ```
    IP=<Turtlebot 3 ip> ./run.sh
    ```