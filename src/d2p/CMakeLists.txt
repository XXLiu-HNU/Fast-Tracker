cmake_minimum_required(VERSION 3.0.2)
project(d2p)

## 使用 C++11 或更新版本
add_compile_options(-std=c++11)

## 查找 catkin 组件和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  vision_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
)

## 查找 Eigen
find_package(Eigen3 REQUIRED)

## 生成 catkin 包
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs 
                 visualization_msgs vision_msgs cv_bridge image_transport 
                 pcl_ros pcl_conversions
  DEPENDS EIGEN3
)

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## 添加可执行文件
add_executable(d2p_node src/main.cpp)

## 指定可执行文件需要链接的库
target_link_libraries(d2p_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
