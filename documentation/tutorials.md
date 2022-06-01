# Tutorials.md

## Table of contents
- [Table of contents](#table-of-contents)
- [Tutorials](#tutorials)
    - [Configuring markdown autoformat in VS Code](#configuring-markdown-autoformat-in-vs-code)
    - [Adding a camera to the turtlebot3 burger model for Gazebo](#adding-a-camera-to-the-turtlebot3-burger-model-for-gazebo)
    - [SLAM in the Gazebo simulation](#slam-in-the-gazebo-simulation)
    - [Nav2 and SLAM on the physical robot](#nav2-and-slam-on-the-physical-robot)
    - [Getting the Raspberry Pi camera working](#getting-the-raspberry-pi-camera-working)
    - [Autonomous exploration in the Gazebo simulator](#autonomous-exploration-in-the-gazebo-simulator)
    - [Autonomous exploration on the physical robot](#autonomous-exploration-on-the-physical-robot)
    - [Add QR code models to Gazebo](#add-qr-code-models-to-gazebo)
    - [Running the nodes](#running-the-nodes)

## Tutorials
### Configuring markdown autoformat in VS Code
1. Install the Markdown All in One extension
    1. Press `Ctrl+Shift+X` to open the extensions tab
    2. Search for `Markdown All in One` and install it
2. Configure VS Code settings
    1. Press `Ctrl+Comma` to open the settings
    2. Search for `format on save` and check the box
    3. Search for `indentation size` and select the option `inherit`
    4. Search for `toc.levels` and change the value to `2..6`

### Adding a camera to the turtlebot3 burger model for Gazebo
1. Add the following lines to `~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`
    ```xml
        <joint name="camera_joint" type="fixed">
          <parent>base_link</parent>
          <child>camera_link</child>
          <pose>0 0 0 0 0 0</pose>
        </joint>

        <link name="camera_link">
          <pose>0.04 0 0.13 0 0 0</pose>
          <inertial>
            <mass>1e-18</mass>
          </inertial>

          <sensor type="camera" name="camera_sensor">
            <update_rate>30</update_rate>
            <camera name="camera_name">
              <horizontal_fov>1.086</horizontal_fov>
              <image>
                <width>1024</width>
                <height>768</height>
                <format>R8G8B8</format>
              </image>
            </camera>

            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
              <alwaysOn>true</alwaysOn>
              <updateRate>0.0</updateRate>
              <camera_name>camera</camera_name>
              <imageTopicName>image_raw</imageTopicName>
              <frame_name>camera_link</frame_name>
              <hack_baseline>0.07</hack_baseline>
            </plugin>
          </sensor>
        </link>
    ```
2. And then recompile the turtlebot3 models
    ```
    cd ~/turtlebot3_ws
    colcon build --symlink-install
    ```
3. Install the v4l2 package
    ```
    apt-get install ros-foxy-v4l2-camera
    ```
4. Run the node
    ```
    ros2 run v4l2_camera v4l2_camera_node
    ```
5. View live feed
    ```
    ros2 run rqt_image_view rqt_image_view
    ```

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

### Getting the Raspberry Pi camera working
1. [Raspi] Download and build the camera node
    1. Add `start_x=1` to the end of `/boot/firmware/config.txt`
    2. Reboot
    3. Run the following commands
        ```
        sudo apt install libtheora-dev libogg-dev libboost-python-dev
        mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
        git clone --branch foxy https://github.com/ros-perception/image_common
        git clone --branch ros2 https://github.com/ros-perception/vision_opencv
        git clone --branch foxy-devel https://github.com/ros-perception/image_transport_plugins
        git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera src/v4l2_camera
        cd ~/ros2_ws
        rosdep install -i --from-path src --rosdistro foxy -y
        cd ~/ros2_ws/src
        rosdep install -i --from-path src --rosdistro foxy -y
        cd ~/ros2_ws
        colcon build --symlink-install
        ```
2. [Raspi] Run the camera node (in a new terminal)
    ```
    cd ~/ros2_ws
    . install/local_setup.bash
    ros2 run v4l2_camera v4l2_camera_node
    ```
    The last command gives errors because apparently, the Raspberry Pi camera doesn't have all the configuration settings a regular camera would have. The errors are probably safe to ignore.
3. [Remote] View the output of the camera
    1.  Run the following command
        ```
        ros2 run rqt_image_view rqt_image_view
        ```
    2.  When rqt has opened, press "Refresh topics"
    3.  Select "/image_raw/compressed" from the dropdown menu 

### Autonomous exploration in the Gazebo simulator
1. Clone and build [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)
    ```
    git clone https://github.com/robo-friends/m-explore-ros2
    cd m-explore-ros2
    colcon build --symlink-install
    ```
2. Run Gazebo, Nav2, and the exploration
    ```
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
    ros2 launch slam_toolbox online_async_launch.py

    cd m-explore-ros2
    source install/setup.bash
    ros2 launch explore_lite explore.launch.py
    ```

### Autonomous exploration on the physical robot
1. [Remote] Clone and build [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2)
    ```
    git clone https://github.com/robo-friends/m-explore-ros2
    cd m-explore-ros2
    colcon build --symlink-install
    ```
2. [Raspi] Run turtlebot3_bringup
    ```
    ros2 launch turtlebot3_bringup robot.launch.py
    ```
3. [Remote] Run Nav2, SLAM, and the exploration
    ```
    ros2 launch turtlebot3_navigation2 navigation2.launch.py
    ros2 launch slam_toolbox online_async_launch.py

    cd m-explore-ros2
    source install/setup.bash
    ros2 launch explore_lite explore.launch.py
    ```

### Add QR code models to Gazebo
1. Copy the models to your Gazebo models directory
    ```
    cp models/qr_code_* ~/.gazebo/models/ -r
    ```

### Running the nodes
1. Go to the workspace in the repository
    ```
    cd workspace
    ```
2. Install dependencies
    ```
    rosdep install -i --from-path src --rosdistro foxy -y
    sudo apt install libzbar-dev
    pip3 install pyzbar
    ```
3. Build
    ```
    colcon build --symlink-install
    ```
4. (In a new terminal window) Source and run
    TODO: Add launch file
    ```
    source install/setup.bash
    ros2 run qr_code_reader launch
    ```