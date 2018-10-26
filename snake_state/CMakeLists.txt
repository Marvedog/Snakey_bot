cmake_minimum_required(VERSION 2.8.3)
project(snake_state)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  message_runtime
  nav_msgs
	geometry_msgs
	snake_msgs
  pkg_utils
  roscpp
  tf
)

# Rostest utilities, multithreading. 
find_package(
	Boost
	REQUIRED
	signals
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	include 
)

install(
	DIRECTORY include/pkg_utils/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# State machine
add_executable(
	state_machine_node
	src/state_machine.cpp
)
target_link_libraries(
	state_machine_node
	${catkin_LIBRARIES}
)
add_dependencies(
	state_machine_node
	${catkin_EXPORTED_TARGETS}
)
