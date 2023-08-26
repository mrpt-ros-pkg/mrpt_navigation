cmake_minimum_required(VERSION 3.8)
project(mrpt_tutorials)

# find dependencies
find_package(ament_cmake REQUIRED)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  datasets
  gazebo
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

#ament_xmllint()
ament_export_dependencies()

# Export old-style CMake variables
#ament_export_include_directories("include/${PROJECT_NAME}")
#ament_export_libraries(${PROJECT_NAME})
# Export modern CMake targets
#ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
