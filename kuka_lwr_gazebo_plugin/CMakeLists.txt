cmake_minimum_required(VERSION 2.8.3)
project(kuka_lwr_gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  gazebo
)

add_definitions (-fpermissive -std=c++11)



#find_package(gazebo REQUIRED)

find_package(kuka_lwr_hw REQUIRED)

MESSAGE( STATUS "--> kuka_lwr_hw_INCLUDE_DIRS : " ${kuka_lwr_hw_INCLUDE_DIRS} )
MESSAGE( STATUS "--> kuka_lwr_hw_LIBRARIES : " ${kuka_lwr_hw_LIBRARIES} )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_lwr_gazebo_plugin
  #CATKIN_DEPENDS kuka_lwr_hw
#  DEPENDS 
#    roscpp
#	gazebo


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${kuka_lwr_hw_INCLUDE_DIRS}
  ${CATKIN_DEVEL_PREFIX}/share/kuka_lwr_hw
)

MESSAGE( STATUS "--> CATKIN_DEVEL_PREFIX: " ${CATKIN_DEVEL_PREFIX} )

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS} ${CATKIN_DEVEL_PREFIX}/lib)

add_library(${PROJECT_NAME} src/lwr_gazebo_plugin.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} kuka_lwr_hw_gazebo )


