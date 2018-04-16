cmake_minimum_required(VERSION 2.8.3)
project(ltm)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED
    roscpp
    rostest
    std_msgs
    warehouse_ros
)

catkin_python_setup()

catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include include
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs
    DEPENDS
)

include_directories(
    ${CATKIN_DEVEL_PREFIX}/include
    include
    ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LINK_DIRS})

