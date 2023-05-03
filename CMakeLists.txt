cmake_minimum_required(VERSION 2.8.3)
project(mav_planner)


# revise https://github.com/flexible-collision-library/fcl/issues/356

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  backward_ros
  libmavconn
  manager_msgs
  tf2_eigen
)


## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

#find_package(fcl QUIET)
#if (fcl_FOUND)
#  # convert cmake target to variables for catkin_package DEPENDS
#  if(TARGET ${FCL_LIBRARIES})
#    get_target_property(LIBFCL_LOCATION ${FCL_LIBRARIES} LOCATION)
#    get_target_property(LIBFCL_INTERFACE_LINK_LIBRARIES ${FCL_LIBRARIES} INTERFACE_LINK_LIBRARIES)
#    get_target_property(LIBFCL_INCLUDE_DIRS ${FCL_LIBRARIES} INTERFACE_INCLUDE_DIRECTORIES)
#    set(LIBFCL_LIBRARIES ${LIBFCL_LOCATION} ${LIBFCL_INTERFACE_LINK_LIBRARIES})
#  else()
#    set(LIBFCL_LIBRARIES ${FCL_LIBRARIES})
#    set(LIBFCL_INCLUDE_DIRS ${FCL_INCLUDE_DIRS})
#  endif()
#else()#
#  find_package(PkgConfig REQUIRED)
#  pkg_check_modules(LIBFCL_PC REQUIRED fcl)
#  set(LIBFCL_INCLUDE_DIRS ${LIBFCL_PC_INCLUDE_DIRS})
#  # find *absolute* paths to LIBFCL_LIBRARIES
#  set(LIBFCL_LIBRARIES)
#  foreach(_lib ${LIBFCL_PC_LIBRARIES})
#    find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
#    list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
#  endforeach()
#endif()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   octomap_msgs   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/path_planner_config.cfg
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs message_runtime message_generation
  DEPENDS octomap # LIBFCL # PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  # ${fcl_INCLUDE_DIRS}
  ${LIBFCL_INCLUDE_DIRS}
)

message("Libs: ${OCTOMAP_LIBRARIES}")

link_libraries(
  ${OCTOMAP_LIBRARIES} 
  ${OMPL_LIBRARY_DIRS}  
  ${fcl_LIBRARY_DIRS}
  )  # 


add_library(${PROJECT_NAME}
  src/path_planner.cpp
)

target_link_libraries(  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${INCLUDE_FILES}
  # ${PROJECT_NAME} # important to find include libraries
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${LIBFCL_LIBRARIES}
  ${fcl_LIBRARIES}
  fcl
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/path_planning.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/nodes/path_planner_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${INCLUDE_FILES}
   ${PCL_LIBRARIES}
   ${PROJECT_NAME} # important to find include libraries
   ${OCTOMAP_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${LIBFCL_LIBRARIES}
   ${fcl_LIBRARIES}
   fcl
)
