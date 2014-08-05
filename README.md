mrpt_navigation
===============

This repository provides packages and tools related to the Mobile Robot Programming Toolkit (MRPT). 
Refer to [http://wiki.ros.org/mrpt_navigation](http://wiki.ros.org/mrpt_navigation) for further documentation.


Compiling
---------

1. Make sure [ROS](http://www.ros.org) is installed in your system! 

2. Make sure to have [MRPT](http://www.mrpt.org) compiled or installed in your system. **Choose one** of the following:
    * Install MRPT from official Debian / Ubuntu repositories: 

             sudo apt-get install libmrpt-dev 
    * Install the latest MRPT version from [PPA repositories](http://www.mrpt.org/MRPT_in_GNU/Linux_repositories).
    * Compile MRPT from sources (see [instructions](http://www.mrpt.org/Building_and_Installing_Instructions)) and either install in system with `sudo make install` or add this to your `~/.bashrc` for CMake to find it:

             export MRPT_DIR=YOUR_MRPT_BUILD_DIR
         
3. Make sure to be familiar with ROS' [catkin build system](http://wiki.ros.org/catkin)!
    * [Oficial tutorials](http://wiki.ros.org/catkin/Tutorials)
    * [J.Bohren's tutorial](http://jbohren.com/articles/gentle-catkin-intro/)

4. Clone this repository in your catkin workspace (and switch to the corresponding branch according to your ROS distro):

        cd ~/ros  # change to match your workspace!!
        cd src
        git clone https://github.com/mrpt-ros-pkg/mrpt_navigation.git

5. For ROS to find these packages, read the catkin documentation and run (or add to your `~/.bashrc`):

        source ~/ros/devel/setup.bash 

6. Build as usual with any other ROS package:

        caktin_make
        catkin_make run_tests # to execute the gtests
        
7. Try the tutorials listed in [http://wiki.ros.org/mrpt_navigation](http://wiki.ros.org/mrpt_navigation)
