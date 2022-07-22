cmake_minimum_required(VERSION 2.8.3)
project(gripper_controller)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")
# ROS



find_package(catkin REQUIRED controller_interface REQUIRED message_generation)

add_service_files(
  DIRECTORY
  srv
  FILES
  gripper_open.srv
  gripper_close.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${GAZEBO_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include  ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/gripper_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${gripper_controller_EXPORTED_TARGETS})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES controller_plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
  PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
