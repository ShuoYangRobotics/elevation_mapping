cmake_minimum_required(VERSION 3.5.1)
project(elevation_mapping_demos)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  elevation_mapping
  grid_map_rviz_plugin
  rviz
  grid_map_filters
  grid_map_ros
  grid_map_core
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS
    include
  #LIBRARIES
  CATKIN_DEPENDS
    elevation_mapping
    grid_map_rviz_plugin
    rviz
  #DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(normal_filter src/normal_filter.cpp)
target_link_libraries(normal_filter
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY config doc launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/ground_truth_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
