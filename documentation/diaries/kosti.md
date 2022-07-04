# Sprint 0
The week was mostly spent trying to install and get ros2 working on my computer and familiarizing myself with the turtlebot and ros2 development environment.
I started the week by trying to install ros2 on my university laptop. I got the listener-publisher working, but due to problems with the university Cubbli distro, 
I could not go further. This issue was solved by installing a fresh Ubuntu system on my computer, after which getting ros2 to work was fairly easy just by following
tutorials. During this time I also assembled the Turtlebot3, which went pain-free. 

The rest of my time was spent getting some simple packages (teleop, and SLAM cartographer) to work with the Gazebo-simulator. While getting the functionality that 
I wanted was easy to get to work, I didn't really feel like I was learning much anything about how ros2 or the turtlebot actually worked.

# Sprint 1
Now that I had managed to get ros2 and some of the basic functionality working, I wanted to delve into the nav2, as it seemed to be one of the most important 
packages for future development. As I had issues with getting my Turtlebot to join my phones wifi hotspot, I initially only focused on the simulator. While this time
too I found it easy to get basic functionality (self-navigation by using cartographed maps) to get working just by copypasting commands from the tutorials. However,
this time I wanted to also understand how nav2 actually works so I started reading about the behaviour trees. Unfortunately I found this task to be too daunting within 
the one week-timeframe while also wanting to get other functionality working. This is the one thing I've found to be difficult in learning ros2. While it is easy 
to get common functionalities to work just by following the tutorials word-for-word, there really isn't an explicit intermediary step between that and actually
starting to develop your own packages and understaning ros2. I'd imagine a good way to do this is to recreate simple versions of already existing packages, but for
now I haven't found the time for that. 

I also got the nav2-SLAM working together and then started to look into how I'd get self-exploration/mapping to work in ros2. Luckily Sami had already found a 
package that did just that, m-explore. This was the first time I had to install and build a package outside the initial ones and there were no issues. 
Lastly, I tried to get these same things working on the physical robot, but the nav2-SLAM doesn't seem to work as well in real physical space as in the simulation.

# Sprint 2

## Nav2 / m-explore

I've been looking into the navigation stack (nav2) and its relation with the explorer_lite node in m-explore by examining the topics and the type of data they publish and subscribe to. Luckily, while the documentation for ros2 is anemic compared to ros documenation, the architecture is documented in the ros version of the explorer_lite node. Even so, the documentation isn't ideal, for example: explore_lite ros version calls an action named move_base. This doesn't exist in ros2 as it is succeeded by nav2, but there is no documentation for how the ros2 explore_lite calls move actions for nav2. Only by looking at the source code and checking nav2 documentation about how move_base has been modified can one get an idea, but even then it's not great. Unfortunately the navigation stack is ridiculously complicated and explorer_lite, as its name suggests, can only really do one thing out of the box and the parameters it provides don't allow for significant behavioral changes. 

I've tried to hack a solution together by resetting the nav2 costmap that explorer_lite uses to calculate the frontiers and give move orders, but I can't seem to make the nav2 service for clearing the costmap to work (ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap). nav2 does show on the logger that this service has been requested but nothing more, and nothing seems to happen. I confirmed this by comparing the data-arrays that the costmap uses, and there are zero changes after calling the clearing service. Only by closing and starting nav2 again resets the costmap. 

I also tried to look into a solution that would circumvent m-explorer completely. This would require understanding nav2 on a much deeper level though and we would have to implement our own algorithm for how the robot explores its surrounding area. I tried testing it with a nav2 python api called simple commander, but unfortunately the ros2 installation that we used doesn't include include simple commander. After installing and building it I ran into a myriad of dependancy issues that don't get solved by rosdep at least easily. I probably could've solved this but I ran out of time.

## Other observations

- I've also noticed that jumping straight into Turtlebot without first e.g. creating your own super simple robot with a URDF-file can be a hassle. Without understanding the basics of how ros2 simulates robots and how it interacts with the robot's parts you can waste time on fairly simple issues.
- https://automaticaddison.com/ provides nice tutorials that try to explain how ros2 works at least to some degree.
- Even though I achieved literally nothing I feel like I learned more this week than the two weeks before it

# Sprint 3

I'm doing this after the fact due to my work week being cut short by covid, so sorry for the brevity. The third sprint proved to be the biggest challenge as we had to move from the third party explorer_lite package to our own package due to explorer_lite not being cofigurable at all. Once we got the node started (Sami initialized it) we managed to add some functionality everyday, which was a big departure from the usual. Biggest things I made during the sprint were the breadth first search, subscription to /tf topic and installing a slightly modified simple commander api from nav2. There was a lot of time spent looking into which topics were relevant to our issues and working with the data we got from them. New issues popped constantly due to nav being  cumbersome, especially with the physical robot, which I don't think we got working in time for the customer meeting.

# Sprint 4

The first half of the sprint was spent with covid, so it took a bit of time to catch up with what we had done once I got back. All in all it was a fairly simple sprint as I mostly just documented and refactored explore_node.
