## Sprint 0 (16.05. - 23.05.)
I hit some obstacles during the installation and was able to determine it was due to the Cubbli Linux distribution used by default at the CS department at the University of Helsinki after one co-student of mine had the same problems while others didn’t. After a fresh install of a base Ubuntu distribution, I was able to proceed with installing ROS2 using the instructions in the official ROS2 documentation (https://docs.ros.org/en/foxy/index.html) and completed it quickly without any problems.

I then jumped right into the tutorials and learned about the CLI tools: turtlesim, rqt, nodes, topics, services, parameters, actions, rqt_console, ROS2 launch, and the recording and playing back of data.  The tutorials were divided into small modules and it was easy to complete them without any previous experience with such frameworks as everything necessary was thoroughly explained. Every new concept is briefly explained, often with visualizing animations, and then put into practice. For me, this is a great way of learning new technologies and things in general.

After learning the initial basics of ROS2 I felt confident in trying to put things into practice and start using the Gazebo simulator. I was able to replicate the teleoperation of the simulated turtlebot3 using my keyboard. This inspired me to try the same with the physical turtlebot3 robot and I was also able to do that without any problems using the turtlebot3 documentation (https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation). 

__Positives__:
 - Easy installation (https://docs.ros.org/en/foxy/Installation.html) 
 - Extensive documentation (https://docs.ros.org/en/foxy/index.html) 
 - Excellent tutorials (https://docs.ros.org/en/foxy/Tutorials.html) 

__Negatives__:
 - None
 
 
## Sprint 1 (24.05. - 30.05.)
I focused mainly on the camera node of the physical TurtleBot3 in this sprint. It took me a lot of time to get the node working and my developing experience reflects this.
 
ROS2 package library (https://index.ros.org/) search seemed somewhat cumbersome at first. In the website GUI, there was no way to specify which ROS2 distribution I wanted the search results to be for. I then noticed the little question mark symbol next to the search bar which lead me to the lunr instruction page (https://lunrjs.com/guides/searching.html) and I was able to specify my searches better. Still, the search didn’t seem to function properly with the term “distro:foxy camera”. This looks for any result with the distro being foxy and any other field being camera. The lunr documentation says that when both fields match, the results will be shown first, but this does not seem to be the case as the correct results are shown on the second page instead of the first. Maybe the index page could have search filtering implemented a bit better. This could, of course, also be my misunderstanding.

In addition to the official ROS documentation and its tutorials, general Googling provides a lot of good results. However, in my opinion, this also highlights a negative aspect of ROS; so many ROS versions where nodes are not cross-compatible. Even when the ROS version is the same as the one used in the tutorial, building often fails. This caused me frustration in this sprint although I was able to get everything working in the end.

I was quite surprised by the very long build times when doing it in the Raspberry Pi. It felt quite frustrating having to wait sometimes up to 2 hours for the packages to build only to have it fail at the last part. But, this is a part of developing with such technologies and is a part of the work process.

The camera package we used (https://index.ros.org/r/v4l2_camera/) has some interesting bugs when we are using it. We can see the camera feed live using ```ros2 run rqt_image_view rqt_image_view``` on the remote pc, but changing the image topic to e.g. /image_raw results in no video shown. Changing back to /image_raw/compressed results in rqt crashing. I must investigate this further.

__Positives__:
 - Lots of premade packages viewable from https://index.ros.org/

__Negatives__:
 - Many tutorials online most of which did not work as is
 - Long build times in Raspberry Pi


## Sprint 2 (31.05. - 07.06.)
In this sprint, my tasks were more broad instead of focusing on one specific aspect like in the previous sprint. I created simulation worlds with QR codes and the launch files for those. I also created an [image for the TurtleBot3](https://drive.google.com/file/d/1JExsfCfhW8HvZbS-rrAKpXwOzQ3-d5AO/view?usp=sharing). We also created launch scripts to make running the nodes much easier and able to be done with one simple command.

After getting to know the Gazebo simulator, creating the test simulation worlds with simple walled structures was quite straightforward. Adding our QR codes to the simulation world proved to be a little bit more tricky. Juho and I proceeded to look at tutorials online and we were able to create a QR code model for the simulator using a picture of the QR code as a texture. After succeeding in this, creating 5 different models for different QR codes was trivial. After adding the QR code models to the world, I noticed that the QR codes are not grouped/joined with the walls of the world itself, and moving the world results in the QR codes staying still and vice versa. Despite this, the test worlds work and I proceeded to more pressing tasks. I might come back to this in the future if moving the test worlds manually is deemed necessary.

An interesting note about the test simulation worlds was that sometimes the robot gets stuck in seemingly large spaces for no reason (our test world has a minimum gap of 1m between walls). This seems to be due to the m-explore-2 package we are using because there is no problem with the same route when manually using nav2.

When we were able to get all the nodes and packages working manually on the TurtleBot3, I created an image for the TurtleBot3 which includes all the progress we have made so far and serves as a checkpoint for us if anything goes wrong with the SD card of the Raspberry Pi. We can easily mount this image and continue working where we left off instead of starting everything from scratch and having to build all the packages we have implemented so far, etc. At the moment the image surely contains some unnecessary elements (though not large in file size) and could be slimmed down, but we decided to leave this be for the moment to ensure that we have a working image. We will come back to this in the next sprint. 

After Juho created some initial launch scripts for the simulator, we started looking into achieving the same result on the TurtleBot3. Juho had previous experience with such scripts which made this a simple task and we were able to do it together quite easily. 

__Positives__:
 - Dealing with nodes and packages is becoming more clear/easy as I get more experience with ROS2
 - Launch scripts make testing simpler

__Negatives__:
 - I feel like I should be getting to know some nodes/packages more in-depth to know how they are working, but I am feeling a bit overwhelmed to do so as we are continuously implementing more and more of them


## Sprint 3 (08.06. - 21.06.)
Our 3rd and so far the longest sprint provided me with an opportunity develop a node from the ground up. I was tasked with creating the I/O node that would be used to control all the functions of the robot; starting/stopping autonomous exploration, viewing observed QR codes, navigating to observed QR codes, stopping the navigation to a QR code, and a manual override using a slightly modified version of the existing [turtlebot3 teleop node](https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_teleop/turtlebot3_teleop/script/teleop_keyboard.py).

To start off I revised [the tutorial of writing a simple publisher and subscriber](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html). By following the tutorial and using the tutorial files ([publisher](https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py), [subscriber](https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py)) as a base, I was able to make a crude version in no time at all. When I was happy with the messages published to the topic, I continued with creating the CLI. This forced me to make some refactoring to adhere to even the most basic concepts of good code. I divided the starting of the node to it's own class and [submodules/views](https://github.com/Le36/ros2-mapper/tree/main/workspace/src/io_node/io_node/submodules) (main menu, QR code menu, manual override menu) to their own classes. Doing this made the node not work anymore. This was due to submodules not inheriting the Node class. Initializing the publishers in the main class and passing the to the submodules as parameters fixed this issue and everything works as intended.

Next up is connecting the I/O node to the other nodes.

Python virtual environment: ROS2 seems to mess up venv when using pip freeze > requirements.txt by e.g. adding relative path to my own computer and adding unnecessary dependencies. After setting up venv building did not work anymore. Following [these instructions](https://docs.ros.org/en/foxy/How-To-Guides/Using-Python-Packages.html) most of the errors were corrected.

__Positives__:
 - Writing a publisher was easy using [the tutorial](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

__Negatives__:
 - TBD