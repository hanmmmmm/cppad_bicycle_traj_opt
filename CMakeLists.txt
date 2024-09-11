cmake_minimum_required(VERSION 3.0.2)
project(cppad_bicycle_traj_opt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cppad_bicycle_traj_opt
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(node_cppad_bicycle_traj_opt 
                src/main.cpp
                src/class_node_traj_opt.cpp
                src/trajectory_solver.cpp)

target_link_libraries(node_cppad_bicycle_traj_opt 
              ${catkin_LIBRARIES}
              m
              ipopt
              )

