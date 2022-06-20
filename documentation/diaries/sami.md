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


## Sprint 2 (31.5. - 7.6.)

During Sprint2 it came obvious that I have to deepen my knowledge about how the nodes and packages we are using are actually build and how they work. Usually the documentation of the packages do not really explain things in detail. It's nice that ros2 has tools that help understanding the packages/nodes/services you are running currently. rqt_graph , rqt and rqt_console are great help to get better understanding. 

A lot of the packages are in C++ which make things troubling as I'm not that familiar with it. So understanding/modifying packages will be a bigger task. 

Most of my time for the week went while building tests and adding functions to our memory node and trying to understand how we could command m-explore. 

There is now tutorials or any advice how the test should be build on ROS2. Mostly I just checked how they were done in other ros2 packages. Only thing mentioned in ros2 tutorials is that they should be in the node own folder. 

## Sprint 3 (8.6. - 21.6.)

During sprint3 we started to create most own the nodes ourselves. We remade explore and camera nodes and created io node. 

I mostly worked on the explore node and it came obvious it wouldn't be as smooth sailing as before. nav2 doesn't have ready made api for python in Foxy. It does have for galactic but we are already heavily invested to use foxy and not changing it at this point of the project. I went trough the simple commander which is the nav2 interface for python in galactic and took the funtions we needed for commanding nav2. They could be used with just small alternations. Later on we ended up alternating the full simple commander for our use. 

Getting exploration working I had to get better understanding of the occupance grid and ended up printing maps in terminal a lot. In the end it was pretty easy to get exploration working at some level with Kosti. We ended up just looking for closest x,y cordinate which isn't explored yet with minimum radius requiment. We also realised loops aren't so easy to use in ros2 and there is really not note on those in tutorials. Getting the exploration and in the end also our movement node working with every other node was the bigger task. 