cmake_minimum_required(VERSION 2.8.3)
project(fcc_serial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

add_definitions(-std=c++11)# -m64) #-Wall)


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
#  LIBRARIES test
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
   sensor_msgs
   cv_bridge
   image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

file(GLOB SOURCES
	src/fcc_serial_node.cpp
)

#add_executable(stereo_vision_node src/stereo_vision_node.cpp)
add_executable(fcc_serial_node ${SOURCES})


target_link_libraries(
	fcc_serial_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}  
        ${Boost_LIBRARIES}
)


