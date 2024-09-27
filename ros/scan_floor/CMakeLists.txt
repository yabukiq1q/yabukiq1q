cmake_minimum_required(VERSION 3.0.2)
project(scan_floor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  turtlesim
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_geometry_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
#find_package(ZeroMQ REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}  # Include ZeroMQ headers
)

add_executable(autonomous_navigation src/autonomousNavigation.cpp)
target_link_libraries(autonomous_navigation
  ${catkin_LIBRARIES}
)

add_executable(serial src/serialCommunication.cpp)
target_link_libraries(serial
  ${catkin_LIBRARIES}
)

add_executable(joy2cmd_vel src/joy2cmd_vel.cpp)
target_link_libraries(joy2cmd_vel
  ${catkin_LIBRARIES}
)

add_executable(pubsubOdom src/pubsubOdom.cpp)
target_link_libraries(pubsubOdom
  ${catkin_LIBRARIES}
  ${ZeroMQ_LIBRARIES}  # Link against ZeroMQ library
)
