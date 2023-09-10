cmake_minimum_required(VERSION 2.8.3)
project(webcam_image_rectifier)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Add ros version check
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_filters image_transport cv_bridge image_geometry nodelet)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS  thread)
include_directories(${Boost_INCLUDE_DIRS})
if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_definitions(-msse3) # not recognized in ARM
endif ()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS  roscpp std_msgs sensor_msgs message_filters image_transport cv_bridge image_geometry nodelet
  #DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
## Your package locations should be listed before other locations

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/sdl2)
message("CMAKE_MODULE_PATH=" ${CMAKE_MODULE_PATH})

#find_package(SDL2 REQUIRED)
#if (SDL2_FOUND)
#  message("SDL2_INCLUDE_DIR=" ${SDL2_INCLUDE_DIR})
#  message("SDL2_LIBRARY=" ${SDL2_LIBRARY})
#  include_directories(${SDL2_INCLUDE_DIR})
#endif (SDL2_FOUND)

#find_package(SDL2_IMAGE REQUIRED)
#if (SDL2_IMAGE_FOUND)
#  message("SDL2_IMAGE_INCLUDE_DIR=" ${SDL2_IMAGE_INCLUDE_DIR})
#  message("SDL2_IMAGE_LIBRARY=" ${SDL2_IMAGE_LIBRARY})
#  include_directories(${SDL2_IMAGE_INCLUDE_DIR})
#endif (SDL2_IMAGE_FOUND)


## Declare a cpp executable
add_executable(webcam_image_rectifier_node src/webcam_image_rectifier_node.cpp src/utils_mei.cpp src/utils_pinhole.cpp src/utils_kannala.cpp src/spline.cpp src/map_interpolator.cpp src/webcam_image_rectifier.cpp)
add_library(webcam_image_rectifier src/webcam_image_rectifier_nodelet.cpp src/utils_mei.cpp src/utils_pinhole.cpp src/utils_kannala.cpp src/spline.cpp src/map_interpolator.cpp src/webcam_image_rectifier.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(webcam_image_rectifier_node  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  yaml-cpp ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARY} ${Eigen3_LIBRARIES})
target_link_libraries(webcam_image_rectifier  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  yaml-cpp ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARY} ${Eigen3_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_node_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS webcam_image_rectifier_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
