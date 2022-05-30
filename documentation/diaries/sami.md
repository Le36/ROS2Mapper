## Sprint 0

### Day 1 16.5.
Realizing my school computer is in really old version of Cubbli Linux. Starting to update it to the latest version 18 → 20. Few problems during the upgrade but getting it through in end of the day. 

During the installation tried to install ros environment in my Windows my computer. Ran into problems while installing Gazebo and as almost all of the instructions are for Linux gave up on the windows as school computer was updated. 

### Day 2 17.5.
Starting to install the ROS2 environment in my Cubbli. Install instructions don’t work directly. Can’t get the library paths and keys working. After some troubleshooting I realize it’s because the created files don’t have right access rights. After modifying the access rights I get environment installed. Same problems with Gazebo installation. Finally got everything working and can connect ros environment via ign bridge to Ignitation Gazebo. 

### Day 3 18.5. 
Start to wonder how to get turtlebot3 model into Gazebo. Couldn’t find any instructions on how to get into Ignitation Gazebo but get it working with old Gazebo rather easily. On installation with the old Gazebo just had to take into account that we are using Foxy version and had to change github branch to Foxy in Gazebo files. 

### Day 4 19.5.
Building the bot. On almost every level forgot to take the wires through before screwing everything in place. Had to multiple times to take screws off take wires through the level and screw the screws back in. 

### Day 5 20.5.
Starting to go through all the tutorials in order from https://docs.ros.org/

### Day 6. 23.5. 
Tutorials are nice but a bit of copy paste tutorials. No tasks that you are meant to do from just instructions. 

## Sprint 1
### Day 7 24.5.
User meeting and sprint planning. No progress on Robot side.

### Day 8 25.5.
Using nav2, cartographer and m-explorer package for mapping and autonomous exploring. Having problems to get exploring working. Giving error about missing /map node. 

### Day 9 27.5.
Group mate got exploring working. Was missing slam_toolbox launch. 

Got camera also working on simulation with v4l2 package. Really straight forward no problems. Group mate already figured how camera can be added on gazebo simulation by modifying model file. 

### Day 10 29.5.
Re-installing ubuntu on SD card. For some reason card stopped working while formatting with pi imager. 

Leave the SD card on table for a while and after that it started working again.

My pre set password didn’t work for some reason after the installation. Also was looking at wrong version of ROS on https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ . The site doesn’t keep version when changing pages and this can cause problems if you forget to change it. Use way too many hours because of this.
