ROS 2 and its installation went smoothly following the instructions: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Gazebo IGN had problems when trying to install a VM running Ubuntu, the problem was finally the vmware gpu driver when the host had a Quadro RTX 3000, the problem disappeared when the gpu got a native driver without vm. On the other hand, ignition was not needed in this project in the end.

The installation of Gazebo 11 went smoothly following the instructions: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation and https://ubuntu.com/blog/simulate-the-turtlebot3

Building the robot was easy, and things related to it made it easy, for example, the ssh connection between devices in the same WLAN and other similar tricks. eg robot control with master keyboard and robot in wireless mode.
