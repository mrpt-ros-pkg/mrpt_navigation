# mrpt_navigation

Metapackage that depends on all the `mrpt_navigation` packages. See the
[repository README](../README.md) for the list of packages and demos.

## End-to-end tests

This package also hosts the end-to-end navigation tests, run headless in the
[mvsim](https://github.com/MRPT/mvsim) simulator with the full stack of
`demo_astar_trajectory_follower_gridmap.launch.py` (map server, A* planner, PF
localization, point cloud pipeline, trajectory follower):

| Test | What it checks |
| --- | --- |
| [test_navigation_battery.py](test/test_navigation_battery.py) | A battery of A->B goals, for the `diffdrive` and `ackermann` robots: no collisions, the robot stays near its path, goals are reached or reported as failed in time, and the localization does not drift. |
| [test_continuous_planning.py](test/test_continuous_planning.py) | Anytime and continuous planning, through the planner `NavigateGoal` action and goal topic. |
| [test_fast_robot_replanning.py](test/test_fast_robot_replanning.py) | Replanning from a robot moving at its top speed. |
| [test_goal_manager_robustness.py](test/test_goal_manager_robustness.py) | The planner goal manager against stale follower status and obstacles that appear on the best path. |
| [test_legacy_follower_status.py](test/test_legacy_follower_status.py) | The goal manager with the legacy (plain string) follower status. |
| [test_navigation_status.py](test/test_navigation_status.py) | The planner status and action feedback. |
| [test_waypoints.py](test/test_waypoints.py) | Waypoint sequences (`NavigateWaypoints` action). |

Run them, from the workspace root, with:

    colcon test --packages-select mrpt_navigation
    colcon test-result --verbose

`test_navigation_battery.py` reads these environment variables:
`MRPT_NAV_TEST_ROBOT` (`diffdrive`, default, or `ackermann`),
`MRPT_NAV_TEST_SPEED_LIMIT` (default: `1.0` m/s), and `MRPT_NAV_TEST_LOG` (a
`.json` file to save the trajectories of each goal, which can be plotted with
[test/plot_nav_traces.py](test/plot_nav_traces.py)).
