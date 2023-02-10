cmake_minimum_required(VERSION 2.8.3)
project(pc_preprocessing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin  REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  
  image_transport
  darknet_ros_msgs
  livox_ros_driver
  cv_bridge
)

find_package( PCL 1.2 REQUIRED )
find_package(OpenCV REQUIRED)
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
#绘图
find_package(PythonLibs 2.7)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/pc_preprocessing
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_LIBS}
  matplotlib-cpp
)

add_library(${PROJECT_NAME}_lib
  src/upsampling.cpp
 
)



add_executable(pc_prep src/pc_preprocessing.cpp)
target_link_libraries(pc_prep ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME}_lib ${PYTHON_INCLUDE_DIRS})



# add_executable(extract_image src/extract_image.cpp include/pc_preprocessing/imageBasics.h)
# target_link_libraries(extract_image ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME}_lib ${PYTHON_INCLUDE_DIRS})


 
