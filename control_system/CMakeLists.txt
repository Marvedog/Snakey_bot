cmake_minimum_required(VERSION 2.8.3)
project(control_system)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pkg_utils
  roscpp
	snake_msgs
	tf
	std_msgs
	nav_msgs
	geometry_msgs
	cmake_modules
	eigen_conversions
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
	wlos_guidance_node
	src/wlos_guidance.cpp
	src/wlos_guidance_node.cpp
	include/control_system/wlos_guidance.h
	include/control_system/wlos_guidance_node.h
)
target_link_libraries(
	wlos_guidance_node
	${catkin_LIBRARIES}
)

add_dependencies(
	wlos_guidance_node
	${catkin_EXPORTED_TARGETS}
)

# Inverse kinematics node
add_executable(
	inverse_kinematics_node
	src/inverse_kinematics.cpp
	include/control_system/inverse_kinematics.h
)
target_link_libraries(
	inverse_kinematics_node
	${catkin_LIBRARIES}
)

add_dependencies(
	inverse_kinematics_node
	${catkin_EXPORTED_TARGETS}
)
