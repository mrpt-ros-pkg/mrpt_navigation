mrpt_common
===========

This stack provides common packages and tools related to the Mobile Robot Programming Toolkit (MRPT). 
Refer to [http://wiki.ros.org/mrpt_common](http://wiki.ros.org/mrpt_common) for further documentation.


Compiling
---------

1. Make sure [ROS](http://www.ros.org) is installed in your system!

2. Make sure to have [MRPT](http://www.mrpt.org) compiled or installed in your system. **Choose one** of the following:
    * Install MRPT from official Debian / Ubuntu repositories: 

             sudo apt-get install libmrpt-dev 
    * Install the latest MRPT version from [PPA repositories](http://www.mrpt.org/MRPT_in_GNU/Linux_repositories).
    * Compile MRPT from sources (see [instructions](http://www.mrpt.org/Building_and_Installing_Instructions)) and either install in system with `sudo make install` or add this to your `~/.bashrc` for pkg-config to find it:

             export PKG_CONFIG_PATH=[YOUR_MRPT_BUILD_DIR]/pkgconfig-no-install:$PKG_CONFIG_PATH
         
         
3. Checkout the stack code from the GitHub repository:

        git clone https://github.com/mrpt-ros-pkg/mrpt_slam.git

4. For ROS to find these packages, append the directory with mrpt-ros-pkg sources to the `ROS_PACKAGE_PATH` variable. Add this to your `~/.bashrc`:

        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[YOUR PATH TO mrpt-ros-pkg/]
        
5. Build as usual with any other ROS package:

        rosmake

