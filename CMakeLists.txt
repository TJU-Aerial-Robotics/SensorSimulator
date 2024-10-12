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
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# CUDA
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_86,code=sm_86")

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
target_link_libraries(sensor_simulator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX yaml-cpp)
target_compile_definitions(sensor_simulator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")


cuda_add_library(raycast_cuda src/sensor_simulator.cu)
target_link_libraries(raycast_cuda ${CUDA_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(sensor_simulator_cuda src/test_simulator_cuda.cpp)
target_link_libraries(sensor_simulator_cuda ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} raycast_cuda yaml-cpp)
target_compile_definitions(sensor_simulator_cuda PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")

