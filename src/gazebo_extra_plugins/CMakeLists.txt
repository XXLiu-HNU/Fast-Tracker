cmake_minimum_required(VERSION 3.0.2)
project(gazebo_extra_plugins)

option(ENABLE_DISPLAY_TESTS "Enable the building of tests that requires a display" OFF)

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        message_generation
        gazebo_msgs
        roscpp
        rospy
        std_msgs
)

include_directories(include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

if (NOT GAZEBO_VERSION VERSION_LESS 6.0)
    catkin_package(  INCLUDE_DIRS include LIBRARIES gazebo_ros_elevator)
endif()

if (NOT GAZEBO_VERSION VERSION_LESS 7.3)
    catkin_package(INCLUDE_DIRS include LIBRARIES gazebo_ros_harness)
endif()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS
        message_runtime
        gazebo_msgs
        roscpp
        rospy
        std_msgs
)

add_library(custom_actor_plugin src/custom_actor_plugin.cpp)
add_dependencies(custom_actor_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_actor_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
    custom_actor_plugin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)


install(DIRECTORY test
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
