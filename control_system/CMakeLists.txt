cmake_minimum_required(VERSION 2.8.3)
project(control_system)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pkg_utils
  roscpp
	snake_msgs
	std_msgs
	nav_msgs
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
  ${Eigen3_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	include 
	/usr/include/eigen3
)

install(
	DIRECTORY include/pkg_utils/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Wlos guidance node
add_executable(
	wlos_guidance	
	src/wlos_guidance.cpp
	src/wlos_guidance_node.cpp
)
target_link_libraries(
	wlos_guidance 
	${catkin_LIBRARIES}
)

add_dependencies(
	wlos_guidance 
	${catkin_EXPORTED_TARGETS}
)
