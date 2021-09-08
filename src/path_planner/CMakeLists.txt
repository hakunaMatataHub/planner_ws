cmake_minimum_required(VERSION 2.8.3)
project(path_planner)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs visualization_msgs turtlesim roscpp)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp std_msgs geometry_msgs visualization_msgs turtlesim)
include_directories(include  ${catkin_INCLUDE_DIRS})



add_executable(planner_interface src/planner_interface.cpp)
target_link_libraries(planner_interface ${catkin_LIBRARIES})

