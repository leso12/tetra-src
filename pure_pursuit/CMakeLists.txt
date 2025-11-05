cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit_local_planner)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  pcl_conversions
  roscpp
  tf
  geometry_msgs
  base_local_planner
  tf2_ros
  tf2_geometry_msgs
)

## Find other dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/PurePursuitPlanner.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pure_pursuit_local_planner
  CATKIN_DEPENDS
    dynamic_reconfigure
    pluginlib
    roscpp
    tf
    tf2_ros
    tf2_geometry_msgs
    costmap_2d
    geometry_msgs
    nav_core
    base_local_planner

)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${Eigen3_DEFINITIONS})

add_library(pure_pursuit_local_planner
  src/pure_pursuit_planner.cpp
  # src/transform_global_plan.cpp
  # src/join_costmap.cpp
  #src/utils.cpp  
)

add_dependencies(pure_pursuit_local_planner
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(pure_pursuit_local_planner
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Install library
install(TARGETS pure_pursuit_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install plugin XML
install(FILES pure_pursuit_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# (선택 사항) Install launch files if any
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

