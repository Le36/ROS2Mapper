### sprint0

ROS 2 and its installation went smoothly following the
instructions: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

Gazebo IGN had problems when trying to install a VM running Ubuntu, the problem was finally the vmware gpu driver when
the host had a Quadro RTX 3000, the problem disappeared when the gpu got a native driver without vm. On the other hand,
ignition was not needed in this project in the end.

The installation of Gazebo 11 went smoothly following the
instructions: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
and https://ubuntu.com/blog/simulate-the-turtlebot3

Building the robot was easy, and things related to it made it easy, for example, the ssh connection between devices in
the same WLAN and other similar tricks. eg robot control with master keyboard and robot in wireless mode.

### sprint1

this sprint i installed the camera system using the following guide:
https://github.com/ros-perception/vision_opencv/tree/ros2/cv_bridge
with opencv and v4l2 camera. getting the camera was simple but colcon build takes a lot of time with sudo upgrades etc.
so lots of waiting when working with these external sensors for the rasp pi.

autonomous driving was working with physical robot with slam toolkit and nav2 where u can point on the map a target for
the robot to move to. getting it working was also quite simple and didnt require anything extra special.

https://github.com/Le36/ros2-mapper/blob/main/documentation/tutorials.md

we created guides for ourselves to follow in the above link. it proved quite useful when working as a team.

so far no programming at all so there isnt much to report etc, just basic installing utilities.

### sprint2

this sprint i created the memory node system for the robot. it was very straightforward process following ros2
tutorials.

i used https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html this guide for it

```Writing a simple publisher and subscriber (Python)```

basically the system creates a topic it listens to and whenever data is published to that topic, it will be added to
sqlite db

system could be extended this way easily in a few different ways. you could just add unique topic for all the db
operations or have one topic handle them with different commands in that range.

what needs to be figured in the next spring is how to publish data from the db, maybe there is a need to refactor this
system into service and client rather than publisher and subscriber system

surprisingly there were no really troubles when developing this node. the tutorial is very straightforward and since
using sqlite u can have db up and running in 10 seconds

looking forward to next week, this sprint was more interesting as there was something to do programming wise albeit how
lite it was

### sprint3

ok this was the big sprint for us. basically in this sprint we are supposed to finish the product. at the moment when
writing this i can say that we are very close!

first i started the sprint with new db schema after customer gave us the requirements for it. it changed a bit along the
sprint but here is the final version of it:

```
,----------------------.                          
|history               |  ,----------------------.
|----------------------|  |qr_codes              |
|id: int               |  |----------------------|
|center_x: float       |  |center_x: float       |
|center_y: float       |  |center_y: float       |
|center_z: float       |  |center_z: float       |
|normal_vector_x: float|  |normal_vector_x: float|
|normal_vector_y: float|  |normal_vector_y: float|
|normal_vector_z: float|  |normal_vector_z: float|
|rotation_w: float     |  |rotation_w: float     |
|rotation_x: float     |  |rotation_x: float     |
|rotation_y: float     |  |rotation_y: float     |
|rotation_z: float     |  |rotation_z: float     |
|time: timestamp       |  |id: int               |
|pk: int               |  `----------------------'
`----------------------'                          
                                                  
                                                  
    ,--------------.                              
    |sqlite_master |                              
    |--------------|                              
    |type: text    |                              
    |name: text    |                              
    |tbl_name: text|                              
    |rootpage: int |                              
    |sql: text     |                              
    `--------------'                              
```

so there is the required history system and the current qr codes that are supposed to be found in the stage.

making this part of the project was just extending the previous memory node that i created.

team refactored the node later on a little and integrated it to exploration node. this was also complicated
because team had problems with the exploration node, also qr codes was changed to aruco markers.

i tried to help with commanding the robot, but the system was confusing for me.

https://automaticaddison.com/navigation-and-slam-using-the-ros-2-navigation-stack/

i tried to follow this guide and somehow modify it, but ended up not getting anything working really.

there were all kinds of problems, even my robot died at one point.

i also created some gazebo simulation worlds for testing purposes, which was fairly straight forward without major
issues.
at some points the team got the automatic exploration working in these simulations, but not on the physical robots.

overall this sprint was the biggest challenge, and it showcased the typical issues you could face in a project like
this.

this is because almost everything we have made is based on some other open source project etc. so its very likely that
even we dont understand everything we have made and refactored into our project. there isnt really anything you could
say that we fully made ourselves and that is just for the nature of the project.
