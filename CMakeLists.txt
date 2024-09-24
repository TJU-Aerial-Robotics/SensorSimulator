cmake_minimum_required(VERSION 3.0.2)
project(sensor_simulator)

set(CMAKE_PREFIX_PATH "/opt/ros/noetic")
set(CMAKE_MODULE_PATH "${CMAKE_PREFIX_PATH}/share/catkin/cmake")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        std_msgs
        sensor_msgs
        cv_bridge
)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
## Declare a catkin package
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sensor_simulator
        CATKIN_DEPENDS roscpp
        DEPENDS OpenCV)

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}  # Eigen库路径
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(sensor_simulator src/test_simulator.cpp src/sensor_simulator.cpp)

# 链接PCL库到可执行文件
target_link_libraries(sensor_simulator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX yaml-cpp)

target_compile_definitions(sensor_simulator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")
