
cmake_minimum_required(VERSION 2.8.3)
project(target_ekf)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  object_detection_msgs
  nav_msgs
  message_filters
  Eigen3 REQUIRED
)


catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS object_detection_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_ekf_node
 src/target_ekf_node.cpp
)

target_link_libraries(target_ekf_node
   ${catkin_LIBRARIES}
)

add_dependencies(target_ekf_node
  ${catkin_EXPORTED_TARGETS}
)


add_executable(target_ekf_sim_node
 src/target_ekf_sim_node.cpp
)

target_link_libraries(target_ekf_sim_node
   ${catkin_LIBRARIES}
)

add_dependencies(target_ekf_sim_node
  ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS
  scripts/detect_ros.py  # 替换为你的 Python 脚本的实际路径
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  # 安装到 ROS 的可执行路径
)