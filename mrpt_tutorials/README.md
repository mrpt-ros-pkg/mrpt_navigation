# mrpt_tutorials

Launch files, configuration files, maps, and Gazebo models for the demos of
the [mrpt_navigation](https://github.com/mrpt-ros-pkg/mrpt_navigation)
packages (ROS 1).

## Demos

Particle filter localization of the robot `r1` in the simulated "gh25" office
environment, using Gazebo. Run each command in a different terminal:

```bash
roslaunch mrpt_tutorials r1_gazebo_gh25.launch
roslaunch mrpt_tutorials r1_localization.launch
roslaunch mrpt_tutorials r1_rviz.launch
roslaunch mrpt_tutorials r1_teleop.launch
```

`r1_localization_real.launch` and `r1_rviz_real.launch` are the equivalent
launch files for the real robot, localizing on `maps/gh25_real_top_laser.simplemap`.

The launch arguments `ini_file`, `map_file`, `robot_name`, and `scan` of the
localization launch files can be used to try other configurations or maps.

## Contents

- `launch/`: Demo launch files and RViz configurations.
- `cfg/`: MRPT INI files for particle filter localization.
- `maps/`: Simple maps (`.simplemap`) of the demo environment.
- `gazebo/`: Gazebo world, robot model (xacro), and meshes.
