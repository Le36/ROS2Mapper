## Sprint 0 (16.05. - 23.05.)
I hit some obstacles during the installation and was able to determine it was due to the Cubbli Linux distribution used by default at the CS department at the University of Helsinki after one co-student of mine had the same problems while others didn’t. After a fresh install of a base Ubuntu distribution I was able to proceed with the installing ROS2 using the instructions in the official ROS2 documentation (https://docs.ros.org/en/foxy/index.html) and completed it quickly without any problems.

I then jumped right into the tutorials and learned about the CLI tools: turtlesim, rqt, nodes, topics, services, parameters, actions, rqt_console, ROS2 launch and the recording and playing back of data.  The tutorials were divided into small modules and it was easy to complete them without any previous experience of such frameworks as everything necessary was thoroughly explained. Every new concept is briefly explained, often with visualizing animations, and then put into practice. For me personally this is a great way of learning new technologies and things in general.

After learning the initial basics of ROS2 I felt confident in trying to put things into practice and start using the Gazebo simulator. I was able to replicate the teleoperation of the simulated turtlebot3 using my own keyboard. This inspired me to try the same with the physical turtlebot3 robot and I was also able to do that without any problems using the turtlebot3 documentation (https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation). 

__Positives__:
 - Easy installation (https://docs.ros.org/en/foxy/Installation.html) 
 - Extensive documentation (https://docs.ros.org/en/foxy/index.html) 
 - Excellent tutorials (https://docs.ros.org/en/foxy/Tutorials.html) 

__Negatives__:
 - None
 
 
## Sprint 1 (24.05. - 30.05.)
I focused mainly on the camera node of the physical Turtlebot3 in this sprint. It took me a lot of time to get the node working and my developing experience reflects this.
 
ROS2 package library (https://index.ros.org/) search seemed somewhat cumbersome at first. In the website GUI there was no way to specify which ROS2 distribution I wanted the search results to be for. I then noticed the little question mark symbol next to the search bar which lead me to the lunr instruction page (https://lunrjs.com/guides/searching.html) and I was able to specify my searches better. Still the search didn’t seem to function properly with the term “distro:foxy camera”. This looks for any result with the distro being foxy and any other field being camera. The lunr documentation says that when both fields match, the results will be shown first, but this does not seem to be the case as the correct results are shown on the second page instead of the first. Maybe the index page could have search filtering implemented a bit better. This could, ofcourse, also be my misunderstanding.

In addition to the official ROS documentation and it's tutorials, general Googling provides a lot of good results. However, in my opinion this also highlights a negative aspect of ROS; so many ROS versions where nodes are not cross-compatible. Even when the ROS version is the same as the one used in the tutorial, building often fails. This caused me frustration in this sprint although I was able to get everything working in the end.

I was quite surprised of the very long build times when doing it in the Raspberry Pi. It felt quite frustrating having to wait sometimes up to 2 hours for the packages to build only to have it fail at the last part. But, this is a part of developing with such technologies and is a part of the work process.

The camera package we used (https://index.ros.org/r/v4l2_camera/) has some imteresting bugs when we are using it. We are able to see the camera feed live using ```ros2 run rqt_image_view rqt_image_view``` on the remote pc, but changing the image topic to e.g. /image_raw results in no video shown. Changing back to /image_raw/compressed results in rqt crashing. I must investigate this further.

__Positives__:
 - Lots of premade packages viewable from https://index.ros.org/

__Negatives__:
 - Many tutorials online most of which did not work as is
 - Long build times in Raspberry Pi
