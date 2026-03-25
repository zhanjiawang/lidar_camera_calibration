cmake_minimum_required(VERSION 3.0.0)
project(lidar_camera_calibration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    message_filters
)

find_package(PCL REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(
    CATKIN_DEPENDS  roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_filters
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/livox_ros_driver
)

add_executable(lidar_camera_calibration
               src/main.cpp)

target_link_libraries(lidar_camera_calibration 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS} 
)
