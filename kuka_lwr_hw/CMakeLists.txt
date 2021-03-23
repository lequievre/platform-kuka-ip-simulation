cmake_minimum_required(VERSION 2.8.3)
project(kuka_lwr_hw)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(gazebo REQUIRED)

add_definitions (-fpermissive -std=c++11)

find_package(catkin REQUIRED COMPONENTS
std_msgs
kdl_parser
transmission_interface
control_toolbox
controller_interface
controller_manager
hardware_interface
realtime_tools
joint_limits_interface
roscpp
tf
urdf
cmake_modules
pluginlib
std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    kdl_parser
    transmission_interface
    control_toolbox
    controller_interface
    controller_manager
    hardware_interface
    realtime_tools
    joint_limits_interface
    roscpp
    tf
    urdf
    cmake_modules
    pluginlib  
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
  )


###########
## Build ##
###########
MESSAGE( STATUS "--> GAZEBO_INCLUDE_DIRS : " ${GAZEBO_INCLUDE_DIRS} )
MESSAGE( STATUS "--> GAZEBO_LIBRARY_DIRS : " ${GAZEBO_LIBRARY_DIRS} )
MESSAGE( STATUS "--> GAZEBO_LIBRARIES : " ${GAZEBO_LIBRARIES} )

MESSAGE( STATUS "--> CATKIN_DEVEL_PREFIX: " ${CATKIN_DEVEL_PREFIX} )
MESSAGE( STATUS "--> CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR} )
MESSAGE( STATUS "--> CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR} )





include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


link_directories(${GAZEBO_LIBRARY_DIRS})

# LWR HW GAZEBO BASE CLASS 
add_library(${PROJECT_NAME}_gazebo SHARED
 src/lwr_hw.cpp
 src/lwr_hw_gazebo.cpp
)

## LWR HW BASE CLASS TO RULE'EM ALL
add_library(${PROJECT_NAME} SHARED
  src/lwr_hw.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


include(ExternalProject)


# Reflexxes type 2 library ----------------------------------------------------------------

message(WARNING "ATTENTION: This project supports the fork of the reflexxes type 2 motion library in https://github.com/lequievre/reflexxes_type2, and install the libraries in your catkin workspace, not in the system")

set(RML_HEADER_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/include/")
set(RML_RELEASE_SHARED_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/release/lib/shared/")
set(RML_DEBUG_SHARED_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/debug/lib/shared/")
set(RML_RELEASE_STATIC_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/release/lib/static/")
set(RML_DEBUG_STATIC_LIBRARY_DIR "${CMAKE_BINARY_DIR}/${PROJECT_NAME}/reflexxes-prefix/src/reflexxes/Linux/x64/debug/lib/static/")

message(STATUS "RML_HEADER_DIR = ${RML_HEADER_DIR}")
message(STATUS "RML_RELEASE_LIBRARY_DIR = ${RML_RELEASE_LIBRARY_DIR}")
message(STATUS "RML_DEBUG_LIBRARY_DIR = ${RML_DEBUG_LIBRARY_DIR}")
message(STATUS "copy dir = ${CATKIN_DEVEL_PREFIX}/include/reflexxes")


ExternalProject_Add(
  reflexxes
  URL "https://github.com/lequievre/reflexxes_type2/blob/master/ReflexxesTypeII.zip?raw=true"
  DOWNLOAD_NAME ReflexxesTypeII.zip
  CONFIGURE_COMMAND ""
  BINARY_DIR reflexxes-prefix/src/reflexxes/Linux
  BUILD_COMMAND make clean64 all64
  INSTALL_COMMAND 
	${CMAKE_COMMAND} -E copy_directory ${RML_HEADER_DIR} ${CATKIN_DEVEL_PREFIX}/include/reflexxes 
	COMMAND 
  	${CMAKE_COMMAND} -E copy ${RML_RELEASE_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/shared/libReflexxesTypeII.so
 	COMMAND
	${CMAKE_COMMAND} -E copy ${RML_RELEASE_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/release/static/libReflexxesTypeII.a
 	COMMAND
	${CMAKE_COMMAND} -E copy ${RML_DEBUG_SHARED_LIBRARY_DIR}/libReflexxesTypeII.so ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/shared/libReflexxesTypeII.so
	COMMAND
	${CMAKE_COMMAND} -E copy ${RML_DEBUG_STATIC_LIBRARY_DIR}/libReflexxesTypeII.a ${CATKIN_DEVEL_PREFIX}/lib/reflexxes/debug/static/libReflexxesTypeII.a
  )
  
include_directories(${CATKIN_DEVEL_PREFIX}/include/RML)

message(WARNING "ATTENTION: This project supports the fork of the Standford library found in https://github.com/lequievre/FRI_IP, and install the libraries in your catkin workspace, not in the system")

ExternalProject_Add(FRILibrary
	GIT_REPOSITORY https://github.com/lequievre/FRI_IP.git
	GIT_TAG master
	BUILD_COMMAND make
	BUILD_IN_SOURCE 1
	CONFIGURE_COMMAND cmake -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
	INSTALL_COMMAND make install
)







