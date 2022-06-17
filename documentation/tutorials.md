# Table of contents

- [VS Code](#vs-code)
  - [Configuring markdown autoformat in VS Code](#configuring-markdown-autoformat-in-vs-code)
- [Remote Ros2 setup](#remote-ros2-setup)
  - [Install Ros2 and Gazebo](#install-ros2-and-gazebo)
  - [Install turtlebot3](#install-turtlebot3)
  - [Install m-explore-ros2](#install-m-explore-ros2)
  - [Setup Ros2](#setup-ros2)
  - [Setup Gazebo](#setup-gazebo)
- [Raspi Ros 2 setup](#raspi-ros-2-setup)
  - [Install Ros 2](#install-ros-2)
  - [Install m-explore-ros2](#install-m-explore-ros2-1)
  - [Setup the turtlebot 3](#setup-the-turtlebot-3)
  - [Setup the Raspberry Pi camera](#setup-the-raspberry-pi-camera)
  - [Add your ssh key to the Raspberry Pi](#add-your-ssh-key-to-the-raspberry-pi)
- [Gazebo](#gazebo)
  - [SLAM in the Gazebo simulation](#slam-in-the-gazebo-simulation)
  - [Autonomous exploration in the Gazebo simulator](#autonomous-exploration-in-the-gazebo-simulator)
- [Physical robot](#physical-robot)
  - [Nav2 and SLAM on the physical robot](#nav2-and-slam-on-the-physical-robot)
  - [Getting the output from the Raspberry Pi camera](#getting-the-output-from-the-raspberry-pi-camera)
  - [Autonomous exploration on the physical robot](#autonomous-exploration-on-the-physical-robot)
- [Project in Gazebo](#project-in-gazebo)
  - [Initialization](#initialization)
  - [Running the project](#running-the-project)
  - [Running the linter](#running-the-linter)
  - [Running the tests](#running-the-tests)
  - [Generating the coverage report](#generating-the-coverage-report)
- [Project on the physical robot](#project-on-the-physical-robot)
  - [Initialization](#initialization-1)
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

## Remote Ros2 setup

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
sed -ie "s/ros2/foxy-devel/g" turtlebot3.repos
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
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/m-explore-ros2/install/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=$((1 + $RANDOM % 232))" >> ~/.bashrc
```

### Setup Gazebo

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
            <camera name='camera_name'>
              <horizontal_fov>1.086</horizontal_fov>
              <image>
                <width>480</width>
                <height>640</height>
                <format>R8G8B8</format>
              </image>
              <clip>
                <near>0.1</near>
                <far>100</far>
              </clip>
              <distortion>
                <k1>0.25106112</k1>
                <k2>-0.6379611</k2>
                <k3>0.40809116</k3>
                <p1>0.0069353</p1>
                <p2>0.01579591</p2>
              </distortion>
              <lens>
                <intrinsics>
                  <fx>525.44920374</fx>
                  <fy>526.37302771</fy>
                  <cx>330.24175119</cx>
                  <cy>243.26842016</cy>
                </intrinsics>
              </lens>
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
2. Recompile the turtlebot3 models
    ```
    cd ~/turtlebot3_ws
    colcon build --symlink-install
    ```
3. Add the QR code models to the Gazebo models directory
    ```
    cp models/qr_code_* ~/.gazebo/models/ -r
    ```

## Raspi Ros 2 setup

- First follow [this](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/) tutorial
    - Remember to select `Foxy` as the version

### Install Ros 2

1. [Raspi] Install packages
    ```
    sudo apt install ros-foxy-desktop python3-colcon-common-extension -y
    sudo apt install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup -y
    ```

### Install m-explore-ros2

1. [Raspi] Install m-explore-ros2
    ```
    cd ~
    git clone https://github.com/robo-friends/m-explore-ros2
    cd m-explore-ros2
    colcon build --symlink-install
    ```

### Setup the turtlebot 3

1. [Raspi] Add the source commands to `~/.bashrc` by running
    ```
    echo "source ~/m-explore-ros2/install/setup.bash" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    ```
2. [Raspi] Edit the `~/.bashrc` file and set `ROS_DOMAIN_ID` to equal the value on the remote PC

### Setup the Raspberry Pi camera

1. [Raspi] Enable the camera
    1. Add `start_x=1` to the end of `/boot/firmware/config.txt`
    2. Reboot

### Add your ssh key to the Raspberry Pi

1. [Remote] Run the following command
    ```
    ssh-copy-id ubuntu@<Turtlebot 3 ip>
    ```

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

### Initialization

1. Create and source venv
    ```
    python3 -m venv venv
    source venv/bin/activate
    ```
2. Make sure that colcon doesn’t try to build the venv
    ```
    touch venv/COLCON_IGNORE
    ```
3. Check that your venv packages are in PYTHONPATH
    ```
    echo $PYTHONPATH
    ```
    This should show the path to your ROS distro, e.g. ```opt/ros/foxy/lib/python3.6/site-packages```, and the path to your environment packages ```.../path-to-your-env/lib/python3.6/site-packages```

    If the env-package path is not present, add it using
    ```
    export PYTHONPATH='.../path-to-your-env/lib/python3.6/site-packages'
    ```

    To avoid adding the env-package path to $PYTHONPATH every single time, ```add export PYTHONPATH='.../path-to-your-env/lib/python3.6/site-packages'``` to your environment activate file.
4. Install the dependencies
    ```
    rosdep install -i --from-path src --rosdistro foxy -y
    pip3 install -r requirements.txt
    ```
5. Build
    ```
    colcon build --symlink-install
    ```

**Note: Remember to source venv before running any of the commands**

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

### Initialization

1. [Raspi] Clone the repository and install the dependencies
    ```
    cd ~
    git clone https://github.com/Le36/ros2-mapper.git
    cd ros2-mapper/workspace
    python3 -m venv venv
    source venv/bin/activate
    pip3 install -r requirements.txt
    rosdep install -i --from-path src --rosdistro foxy -y
    ```

### Running the project

1. [Remote] Find out the IP of the Raspberry Pi
2. [Remote] Run the launch script
    ```
    IP=<Turtlebot 3 ip> ./run.sh
    ```