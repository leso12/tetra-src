cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_pkg)

## Compile as C++11, if available
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  xacro
  urdf
  robot_state_publisher
  controller_manager
  diff_drive_controller
  joint_state_controller
  transmission_interface
)

## Uncomment this if you have defined a package information file
# set(CFG_INSTALL_SETTINGS
#   INSTALL_SCHEME "DEB"
#   INSTALL_PREFIX "${CATKIN_DEVEL_PREFIX}"
#   INSTALL_PRIVATE_PREFIX "${CATKIN_DEVEL_PREFIX}"
#   INSTALL_INCLUDES_DIR "include"
#   INSTALL_LIBRARIES_DIR "lib"
#   INSTALL_ARCHIVE_DIR "lib"
#   INSTALL_RUNTIME_DIR "lib"
#   INSTALL_PACKAGE_SHARE_DIR "share/${PROJECT_NAME}"
#   INSTALL_PYTHON_DIR "lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages/${PROJECT_NAME}"
# )

###################################
## catkin specific configuration ##
###################################
## The catkin package name and the project name must be the same
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    gazebo_ros
    xacro
    robot_state_publisher
    controller_manager
    diff_drive_controller
    joint_state_controller
    transmission_interface
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package includes headers in the include directory.
# include_directories(include)

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}.cpp
# )

## Declare a C++ executable
## If you have executable C++ nodes in your package,
## add their source files here.
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Add cmake target dependencies of the executable here
## as well as the library.
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a Python executable
## If you have executable Python nodes in your package,
## add a script here.
# catkin_install_python(PROGRAMS scripts/my_python_node
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# install(PROGRAMS scripts/my_robot_teleop.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Do not install C++ libraries if you do not have any
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based C++ tests if catkin comes with gtest
## Please consult the catkin documentation for more information on how to specify tests
# catkin_add_gtest(${PROJECT_NAME}-test src/test_something.cpp)
