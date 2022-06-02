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

I've been looking into the navigation stack (nav2) and its relation with the explorer_lite node in m-explore by examining the topics and the type of data they publish and subscribe to. Luckily, while the documentation for ros2 is anemic compared to ros documenation, the architecture is documented in the ros version of the explorer_lite node. Unfortunately the navigation stack is ridiculously complicated and explorer_lite, as its name suggests, can only really do one thing out of the box and the parameters it provides don't allow for significant behavioral changes. Either we have to change the explorer_lite to some other autonomous path finding node or we have to manually manipulate the topic messages and maybe even the code. 

I've also noticed that jumping straight into Turtlebot, which is a fairly complicated robot without first e.g. creating your own super simple robot with a URDF-file can be a hassle. Without understanding the basics of how ros2 can simulate robots and how it interacts with the robot's parts you can waste time on fairly simple issues.
