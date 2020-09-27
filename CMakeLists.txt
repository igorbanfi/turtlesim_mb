cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_mb)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_generation
rosconsole roscpp
roscpp_serialization
roslib rostime
std_msgs
std_srvs
tf
camera_info_manager
image_transport
cv_bridge
dynamic_reconfigure
gazebo_plugins
gazebo_ros
gazebo_ros_control
geometry_msgs
nav_core
nav_msgs
costmap_2d
)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
                  Color.msg Pose.msg)

add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(turtlesim_mb_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)

set(turtlesim_mb_node_HDRS
  include/turtlesim_mb/turtle_frame.h
)

qt5_wrap_cpp(turtlesim_mb_node_MOCS ${turtlesim_mb_node_HDRS})

add_executable(turtlesim_mb_node ${turtlesim_mb_node_SRCS} ${turtlesim_mb_node_MOCS})
target_link_libraries(turtlesim_mb_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtlesim_mb_node turtlesim_mb_gencpp)

install(TARGETS turtlesim_mb_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
