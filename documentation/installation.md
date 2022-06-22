# Installation
## Installing dependencies
1. [Remote] Install Ros2 and Gazebo
    ```
    sudo apt install ros-foxy-desktop python3-colcon-common-extension -y
    sudo apt install gazebo11 ros-foxy-gazebo-ros-pkgs -y
    sudo apt install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup -y
    ```
2. [Remote] Install TurtleBot3
    ```
    sudo apt install python3-vcstool
    mkdir -p ~/turtlebot3_ws/src
    cd ~/turtlebot3_ws
    wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
    sed -ie "s/ros2/foxy-devel/g" turtlebot3.repos
    vcs import src < turtlebot3.repos
    colcon build --symlink-install
    ```
3. [Remote] Setup Ros2
    Add the source commands to `~/.bashrc` by running
    ```
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
    ```
4. [Remote] Setup Gazebo
    1. Add the following lines to `~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf`
        ```xml
      <joint name='camera_joint' type='fixed'>
        <parent>base_link</parent>
        <child>camera_link</child>
        <pose>0 0 0 0 -0 0</pose>
      </joint>
      <link name='camera_link'>
        <pose>0.04 0 0.13 0 -0 0</pose>
        <inertial>
          <mass>1e-18</mass>
          <pose>0 0 0 0 -0 0</pose>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <sensor name='camera_sensor' type='camera'>
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
          <plugin name='camera_controller' filename='libgazebo_ros_camera.so'>
            <alwaysOn>1</alwaysOn>
            <updateRate>0.0</updateRate>
            <camera_name>camera</camera_name>
            <imageTopicName>image_raw</imageTopicName>
            <frame_name>camera_link</frame_name>
            <hack_baseline>0.07</hack_baseline>
          </plugin>
        </sensor>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
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
5. [TurtleBot3] Initial setup
    1. Build the robot
    2. Follow [this](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/) tutorial and remember to select `Foxy` as the version
6. [TurtleBot3] Install Ros 2
    ```
    sudo apt install ros-foxy-desktop python3-colcon-common-extension -y
    sudo apt install ros-foxy-cartographer ros-foxy-cartographer-ros ros-foxy-navigation2 ros-foxy-nav2-bringup -y
    ```
7. [TurtleBot3] Setup the turtlebot 3
    1. Add the source commands to `~/.bashrc` by running
        ```
        echo "source ~/m-explore-ros2/install/setup.bash" >> ~/.bashrc
        echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
        ```
    2. Edit the `~/.bashrc` file and set `ROS_DOMAIN_ID` to equal the value on the remote PC
8. [TurtleBot3] Setup the Raspberry Pi camera
    1. Add `start_x=1` to the end of `/boot/firmware/config.txt`
    2. Reboot
9.  [Remote] Add your ssh key to the Raspberry Pi
    1. Generate an ssh key if you already don't have one
    2. Add the public key to the TurtleBot3
        ```
        ssh-copy-id ubuntu@<TurtleBot 3 ip>
        ```

## Installing project
1. [Remote] Clone the repository and go to the workspace
    ```
    git clone https://github.com/Le36/ros2-mapper.git
    cd ros2-mapper/workspace
    ```
2. [Remote] Create the virtual environment
    1. Create the virtual environment
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
        echo $PYTHONPATH | sed -e "s/:/\n/g"
        ```
        - This should show the path to your ROS distro, e.g. `opt/ros/foxy/lib/python3.8/site-packages`, and the path to your environment packages `path-to-your-env/lib/python3.8/site-packages`

        - If the env-package path is not present, add it using
            ```
            export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.8/site-packages
            ```
        - To avoid adding the env-package path to $PYTHONPATH every single time, add the following line to `venv/bin/activate`
            ```
            export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.8/site-packages
            ``` 
3. [Remote] Install the dependencies
    ```
    rosdep install -i --from-path src --rosdistro foxy -y
    pip3 install -r requirements.txt
    ```
    - Installing the requirements with pip might give the error `ERROR: Failed building wheel for empy`, but it is probably safe to ignore
4. [Remote] Build
    ```
    colcon build --symlink-install
    ```
5. [TurtleBot3] Clone the repository
    ```
    cd ~
    git clone https://github.com/Le36/ros2-mapper.git
    cd ros2-mapper/workspace
    ```
6. [TurtleBot3] Create the virtual environment
    1. Create the virtual environment
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
        echo $PYTHONPATH | sed -e "s/:/\n/g"
        ```
        - This should show the path to your ROS distro, e.g. `opt/ros/foxy/lib/python3.8/site-packages`, and the path to your environment packages `path-to-your-env/lib/python3.8/site-packages`

        - If the env-package path is not present, add it using
            ```
            export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.8/site-packages
            ```
        - To avoid adding the env-package path to $PYTHONPATH every single time, add the following line to `venv/bin/activate`
            ```
            export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.8/site-packages
            ``` 
7.  [TurtleBot3] install the dependencies
    ```
    pip3 install -r requirements.txt
    rosdep install -i --from-path src --rosdistro foxy -y
    ```
    - Installing the requirements with pip might give the error `ERROR: Failed building wheel for empy`, but it is probably safe to ignore
