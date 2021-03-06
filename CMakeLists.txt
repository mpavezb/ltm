cmake_minimum_required(VERSION 2.8.3)
project(ltm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
find_package(catkin REQUIRED
    roscpp
    rostest
    std_srvs
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    ltm_db
    pluginlib
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    Date.msg
    EmotionalRelevance.msg
    EntityLog.msg
    EntityMetadata.msg
    EntityRegister.msg
    Episode.msg
    HistoricalRelevance.msg
    Info.msg
    QueryResult.msg
    Relevance.msg
    StreamMetadata.msg
    StreamRegister.msg
    What.msg
    When.msg
    Where.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    AddEpisode.srv
    DropDB.srv
    GetEpisodes.srv
    GetEntityLogs.srv
    QueryServer.srv
    RegisterEpisode.srv
    SwitchDB.srv
    UpdateTree.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include include
    LIBRARIES ltm_plugins
    CATKIN_DEPENDS roscpp std_srvs std_msgs geometry_msgs sensor_msgs message_runtime ltm_db
    DEPENDS
)


###########
## Build ##
###########
include_directories(
    ${CATKIN_DEVEL_PREFIX}/include
    include
    ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LINK_DIRS})

add_library(ltm_plugins
    src/plugin/emotion_manager.cpp
    src/plugin/location_manager.cpp
    src/plugin/streams_manager.cpp
    src/plugin/entities_manager.cpp
    src/plugin/plugins_manager.cpp
)
add_dependencies(ltm_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ltm_server
    src/server.cpp
    src/db/episode_collection.cpp
    src/db/episode_metadata.cpp
    src/db/episode_updater.cpp
    src/util/geometry.cpp
    src/util/util.cpp
)
add_dependencies(ltm_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ltm_server ltm_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hola.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
