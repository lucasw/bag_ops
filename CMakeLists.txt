cmake_minimum_required(VERSION 2.8.3)
project(bag_ops)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbag_storage
  roscpp
  roslint
  rospy
  std_msgs
)

set(ROSLINT_PYTHON_OPTS "--ignore=W503,E203" "1>&2")
roslint_python()

set(ROSLINT_CPP_OPTS
  "--extensions=cpp,h,hpp" "--filter=-whitespace/braces,-runtime/references,-build/c++11"
)
roslint_cpp()

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bag_player
  src/bag_player.cpp
)
target_link_libraries(bag_player
  ${catkin_LIBRARIES}
)

install(TARGETS bag_player
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
