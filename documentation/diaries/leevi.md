sprint9

ROS 2 and its installation went smoothly following the instructions: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Gazebo IGN had problems when trying to install a VM running Ubuntu, the problem was finally the vmware gpu driver when the host had a Quadro RTX 3000, the problem disappeared when the gpu got a native driver without vm. On the other hand, ignition was not needed in this project in the end.

The installation of Gazebo 11 went smoothly following the instructions: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation and https://ubuntu.com/blog/simulate-the-turtlebot3

Building the robot was easy, and things related to it made it easy, for example, the ssh connection between devices in the same WLAN and other similar tricks. eg robot control with master keyboard and robot in wireless mode.


sprint1
this sprint i installed the camera system using the following guide:
https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge
with opencv and v4l2 camera. getting the camera was simple but colcon build takes a lot of time with sudo upgrades etc. so lots of waiting when working with these external sensors for the rasp pi.

autonomous driving was working with physical robot with slam toolkit and nav2 where u can point on the map a target for the robot to move to. getting it working was also quite simple and didnt require anything extra special.

https://github.com/Le36/ros2-mapper/blob/main/documentation/tutorials.md

we created guides for ourselves to follow in the above link. it proved quite useful when working as a team.

so far no programming at all so there isnt much to report etc, just basic installing utilities.
