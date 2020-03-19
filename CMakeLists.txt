cmake_minimum_required(VERSION 2.8.3)
project(mapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
  message_generation
  cmake_modules
  std_srvs
  pensa_msgs
  roslint)

find_package(
  Eigen REQUIRED)

find_package(
  octomap REQUIRED)

find_package(
  Boost COMPONENTS thread)

link_directories(
    ${PCL_LIBRARY_DIRS})

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS
    roscpp
    message_runtime
    cmake_modules
)

roslint_cpp()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS
    roscpp
    message_runtime
    cmake_modules
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

set( LIBS_TO_LINK
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
) 

add_executable(mapper
  src/mapper.cpp
  src/mapper_class.cpp
  src/polynomials.cpp
  src/sampled_trajectory.cpp
  src/octoclass.cpp
  src/msg_conversions.cpp
  src/threads.cpp
  src/visualization_functions.cpp
  src/msg_conversions.cpp
  src/callbacks.cpp
  src/services.cpp
  src/graphs.cpp
  src/rrt.cpp
  src/prm.cpp
  src/rrg.cpp
  src/octopath.cpp
)
target_link_libraries( mapper
    ${LIBS_TO_LINK}
)

#This makes sure that messages and services are compiled before the rest
add_dependencies(mapper mapper_generate_messages_cpp 
                        pensa_msgs_generate_messages_cpp
                        ${catkin_EXPORTED_TARGETS})