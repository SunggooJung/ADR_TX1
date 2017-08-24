SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
cmake_minimum_required(VERSION 2.8.3)
project(optical_flow)

find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

add_definitions(-std=c++11)# -m64) #-Wall)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(

  CATKIN_DEPENDS roscpp std_msgs
   opencv
   sensor_msgs
   cv_bridge
   image_transport
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

file(GLOB SOURCES
	src/optical_flow.cpp
)

add_executable(optical_flow ${SOURCES})

target_link_libraries(
	optical_flow
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS} 
)



