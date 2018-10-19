cmake_minimum_required(VERSION 2.8.3)
project(snake_viz)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  roscpp
	tf
	pkg_utils
  snake_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
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

# path viz node
add_executable(
	path_visualization_node
	src/path_visualization.cpp
)
target_link_libraries(
	path_visualization_node
	${catkin_LIBRARIES}
)
add_dependencies(
	path_visualization_node
	${catkin_EXPORTED_TARGETS}
)
# guidance viz node
add_executable(
	guidance_visualization_node
	src/guidance_visualization.cpp
)
target_link_libraries(
	guidance_visualization_node
	${catkin_LIBRARIES}
)
add_dependencies(
	guidance_visualization_node
	${catkin_EXPORTED_TARGETS}
)
add_executable(
	snake_visualization_node
	src/snake_visualization.cpp
	include/snake_viz/snake_visualization.h
)
target_link_libraries(
	snake_visualization_node
	${catkin_LIBRARIES}
)
add_dependencies(
	snake_visualization_node
	${catkin_EXPORTED_TARGETS}
)
