# Diary, Juho

## Sprint 0
### Monday 16.05
I use Arch Linux (btw) on my laptop so I started trying to install gazebo on it. There is a [gazebo package](https://aur.archlinux.org/packages/gazebo) in the Arch User Repository but installing it wasn't easy. I had to install some of the dependencies separately because their PKGBUILDs had errors and one of them even had a compile error, which I fixed by editing the file just before it was compiled. (Not the correct way to fix things, but it worked)  
After I installed Gazebo, I started trying to install ROS 2. The Galactic version of ROS 2 is also a [package](https://aur.archlinux.org/packages/ros2-galactic) in the Arch User Repository, but it had too many errors and I gave up trying to install it.

### Tuesday 17.05
I tried to install ROS 2 Foxy by compiling from source using [this guide](https://wiki.archlinux.org/title/ROS#Building_from_source), but that also had compile errors. I don't remember If I managed to compile it, but there were many problems and ROS 2 doesn't officially support Arch.  
I found a [good tutorial](https://ubuntu.com/blog/simulate-the-turtlebot3) for installing ROS 2 and gazebo on Ubuntu and tried it out on a Ubuntu virtual machine. The tutorial only had a couple of errors, but fixing them was easy. Now I had Gazebo and ROS 2 running on a Ubuntu virtual machine.

### Wednesday 18.05
I don't want to develop code and run the simulation on a virtual machine so I decided to install Ubuntu on my laptop beside Arch. After installing and configuring Ubuntu to my liking, I installed gazebo and ROS 2 following the tutorial and it worked out nicely.

### Thursday 19.05
I started reading the `Beginner: CLI Tools` part of the [ROS 2 tutorial](https://docs.ros.org/en/foxy/Tutorials.html) to get an understanding of how it works. I learned what types of nodes are possible and how they can communicate, but I'm pretty sure I won't remember the commands because I just copy-pasted them to my console. I liked the visualizations that the tutorials had, and they helped me to understand the subject at hand. I also wondered why `ros2 run turtlesim turtle_teleop_key` uses so much CPU.  
I also had a problem with the `rqt*` programs where the icons were not displayed. This is probably because I'm using [i3 window manager](https://i3wm.org/) instead of Gnome. My `rqt_graph` also looked different than in the tutorial (the topics didn't have rectangles around them). (Future note: after configuring the settings to be the same as in the tutorial, it looked the same)

### Friday 20.05
I started reading the Python part of the `Beginner: Client Libraries` part of the ROS 2 tutorial. The tutorial always said to `fill in the <description>, <maintainer> and <license> tags`, but this was a tutorial and that was not necessary? The tutorial was also basically just copy-pasting code, so I didn't learn too much. At least I can recheck the tutorial when I need to make some code of my own.
After reading the tutorial, I made some testing code of my own and started thinking about possible architectures for our project. Writing the code was a bit confusing, but copy-pasting code from the tutorials and then editing it worked out nicely.  
But now I am confused about the dependencies because one of my nodes was missing `<exec_depend>rclpy</exec_depend>` from the `package.xml` file, but the project still worked.

### Saturday 21.05
When looking up how to find QR codes from images and get their relative position, I found a [GitHub repository](https://github.com/rudzen/qr_radar2) that contained a ROS ~~2~~ node that automatically found QR codes and told the distance and angle relative to them. (Future note: It was a ROS 1 node, and I'm not sure if we can use it in ROS 2)

### Monday 23.05
I started trying to figure out how I would attach add a camera to the TurtleBot3 in Gazebo but didn't get too far.  
I also tried to figure out how to use Nav2 in the simulation but the [Nav2 tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html) didn't work.  

## Sprint 1
