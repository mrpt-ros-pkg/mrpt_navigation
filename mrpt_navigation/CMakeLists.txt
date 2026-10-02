cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

project(mrpt_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # End-to-end navigation tests in simulation (mvsim, headless): a battery of
  # A->B goals for each simulated robot, checking there are no collisions and
  # goals are reached or reported as failed in time (see the test criteria).
  find_package(ament_cmake_ros REQUIRED)
  foreach(ROBOT diffdrive ackermann)
    ament_add_ros_isolated_pytest_test(test_navigation_battery_${ROBOT}
      test/test_navigation_battery.py
      TIMEOUT 1200
      ENV MRPT_NAV_TEST_ROBOT=${ROBOT}
    )
  endforeach()
endif()

# The following is needed for ROS 2's ament to generate the setup hooks
ament_package()