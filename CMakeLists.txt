cmake_minimum_required(VERSION 2.8.3)
project(sm_project)	


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  move_base_msgs
  actionlib
  actionlib_msgs
)

#find_package(Eigen3 REQUIRED)
find_package(Eigen3 QUIET)
#find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

#find_package(PCL 1.7 REQUIRED)

#add_service_files(
   #FILES
   #GoTargetName.srv
   #Approach.srv
#)

#generate_messages(
   #DEPENDENCIES
   #std_msgs
   #std_srvs
#)

include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(include)
#include_directories(src)

catkin_package(
  #INCLUDE_DIRS include
  #INCLUDE_DIRS include
  DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  tf
  actionlib
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#add_executable(nav_service_test src/service_test.cpp ${SOURCES})
#target_link_libraries(nav_service_test ${catkin_LIBRARIES})
#add_dependencies(nav_service_test ${catkin_EXPORTED_TARGETS} villa_navi_service_gencpp)


install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)

