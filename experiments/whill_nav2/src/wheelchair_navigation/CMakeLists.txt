cmake_minimum_required(VERSION 3.8)
project(wheelchair_navigation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY profiles launch behavior_trees motion_primitives config calibration docs scripts visuals
  DESTINATION share/${PROJECT_NAME})
ament_package()
