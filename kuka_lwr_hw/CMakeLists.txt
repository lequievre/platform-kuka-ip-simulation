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

