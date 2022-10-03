cmake_minimum_required(VERSION 3.8)
project(mrpt_tutorials)

# find dependencies
find_package(ament_cmake REQUIRED)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  cfg
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

ament_package()
