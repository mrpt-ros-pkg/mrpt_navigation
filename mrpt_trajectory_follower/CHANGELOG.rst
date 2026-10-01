^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_trajectory_follower
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.6.0 (2026-10-01)
------------------
* Merge pull request `#175 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/175>`_ from mrpt-ros-pkg/mrpt3
* Merge ros2 into mrpt3
* trajectory_follower: update sample config comments for the vehicle-centered control-pose filter
* trajectory_follower: document tight anchor_max_ang_divergence for jittery localization
* mrpt_trajectory_follower: document new TrajectoryFollower params
* trajectory_follower: wire follower_debug_trace param to logger verbosity
* add new upstream params
* follower params: switch to curvature-adaptive lookahead knobs
* Merge pull request `#172 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/172>`_ from mrpt-ros-pkg/fix/follower-cmd-vel-idle-silence
* trajectory_follower: only publish cmd_vel while actively driving
* update to use new mpp lib structure
* fix: mpp follow lib header reorganization
* Merge pull request `#171 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/171>`_ from mrpt-ros-pkg/feat/follower-arrival-radius
* feat(trajectory_follower): expose arrival_radius param
* Merge pull request `#170 <https://github.com/mrpt-ros-pkg/mrpt_navigation/issues/170>`_ from mrpt-ros-pkg/feat/trajectory-follower-node
* fix cmake linter and tolerances
* fix(trajectory_follower): address code review
* build(trajectory_follower): guard node on mpp::TrajectoryFollower header
* feat(trajectory_follower): 3D-lidar obstacle conditioning, path dedup
* feat(trajectory_follower): new node wrapping mpp::TrajectoryFollower
* Contributors: Jose Luis Blanco-Claraco
