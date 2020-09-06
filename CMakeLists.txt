CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
project(rviz_hitl_slam_tool)

FIND_PACKAGE(Qt5 COMPONENTS Core Gui REQUIRED)
FIND_PACKAGE(catkin COMPONENTS message_runtime message_generation std_msgs roscpp rviz geometry_msgs)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
#SET(CMAKE_AUTOMOC ON)
SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

add_message_files(DIRECTORY msg FILES HitlSlamInputMsg.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

qt5_wrap_cpp(MOC_FILES src/hitl_slam_tool.h)

catkin_package(
  LIBRARIES rviz_hitl_slam_tool
  CATKIN_DEPENDS message_runtime std_msgs rviz geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})
add_library(rviz_hitl_slam_tool src/hitl_slam_tool.cc ${MOC_FILES})
target_link_libraries(rviz_hitl_slam_tool Qt5::Core Qt5::Gui)
add_dependencies(rviz_hitl_slam_tool rviz_hitl_slam_tool_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


