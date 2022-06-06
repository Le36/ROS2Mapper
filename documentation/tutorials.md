# Table of contents

- [VS Code](#vs-code)
    - [Configuring markdown autoformat in VS Code](#configuring-markdown-autoformat-in-vs-code)
- [Ros2 setup](#ros2-setup)
    - [Install Ros2 and Gazebo](#install-ros2-and-gazebo)
    - [Install turtlebot3](#install-turtlebot3)
    - [Install m-explore-ros2](#install-m-explore-ros2)
    - [Setup Ros2](#setup-ros2)
- [Gazebo](#gazebo)
    - [Adding a camera to the turtlebot3 burger model for Gazebo](#adding-a-camera-to-the-turtlebot3-burger-model-for-gazebo)
    - [SLAM in the Gazebo simulation](#slam-in-the-gazebo-simulation)
    - [Autonomous exploration in the Gazebo simulator](#autonomous-exploration-in-the-gazebo-simulator)
    - [Adding the QR code models to Gazebo](#adding-the-qr-code-models-to-gazebo)
- [Physical robot](#physical-robot)
    - [Nav2 and SLAM on the physical robot](#nav2-and-slam-on-the-physical-robot)
    - [Getting the Raspberry Pi camera working](#getting-the-raspberry-pi-camera-working)
    - [Autonomous exploration on the physical robot](#autonomous-exploration-on-the-physical-robot)
- [Project](#project)
    - [Initialization](#initialization)
    - [Running the project](#running-the-project)
    - [Running the linter](#running-the-linter)
    - [Running the tests](#running-the-tests)

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

## Ros2 setup

### Install Ros2 and Gazebo

```
sudo apt install ros-foxy-desktop python3-colcon-common-extension -y
sudo apt install gazebo11 ros-foxy-gazebo-ros-pkgs -y
sudo apt install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup -y
```

### Install turtlebot3

```
sudo apt install python3-vcstool
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
sed -ie 's/ros2/foxy-devel/g' turtlebot3.repos
vcs import src < turtlebot3.repos
colcon build --symlink-install
```

### Install m-explore-ros2

```
cd ~
git clone https://github.com/robo-friends/m-explore-ros2
cd m-explore-ros2
colcon build --symlink-install
```

### Setup Ros2

Add the source commands to `~/.bashrc` by running
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'source ~/m-explore-ros2/install/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo "export ROS_DOMAIN_ID=$((1 + $RANDOM % 232))" >> ~/.bashrc
```

## Gazebo

### Adding a camera to the turtlebot3 burger model for Gazebo

1. Add the following lines
   to `~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`
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

### Autonomous exploration in the Gazebo simulator

Run Gazebo, Nav2, and the exploration
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py
ros2 launch explore_lite explore.launch.py
```

### Adding the QR code models to Gazebo

1. Copy the models to your Gazebo models directory
    ```
    cp models/qr_code_* ~/.gazebo/models/ -r
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
   The last command gives errors because apparently, the Raspberry Pi camera doesn't have all the configuration settings
   a regular camera would have. The errors are probably safe to ignore.
3. [Remote] View the output of the camera
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

## Project

**Note: The following commands must be run in the workspace folder of the repository**

### Initialization

1. Install the dependencies
    ```
    rosdep install -i --from-path src --rosdistro foxy -y
    sudo apt install pycodestyle libzbar-dev python3-opencv -y
    pip3 install pyzbar pytest-env
    pip3 install pytest -U
    ```
2. Build
    ```
    colcon build --symlink-install
    ```


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
