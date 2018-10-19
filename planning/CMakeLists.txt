cmake_minimum_required(VERSION 2.8.3)
project(planning)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pkg_utils
  roscpp
  roslib
	snake_msgs
	std_msgs
	nav_msgs
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

# Spline generator node
add_executable(
	spline_generator	
	src/spline_generator.cpp
	src/spline_generator_node.cpp
)
target_link_libraries(
	spline_generator	
	${catkin_LIBRARIES}
)

add_dependencies(
	spline_generator	
	${catkin_EXPORTED_TARGETS}
)

# Waypoint provider node
add_executable(
		${PROJECT_NAME}_waypoint_provider
		src/waypoint_provider.cpp 
		src/waypoint_provider_node.cpp 
		include/planning/waypoint_provider.h 
		include/planning/csv.h 
)
target_link_libraries(
		${PROJECT_NAME}_waypoint_provider 
		${catkin_LIBRARIES}
)
add_dependencies(
		${PROJECT_NAME}_waypoint_provider
		${catkin_EXPORTED_TARGETS}
)
# Mock class
add_executable(
		${PROJECT_NAME}_mock
		src/mock.cpp 
)
target_link_libraries(
		${PROJECT_NAME}_mock
		${catkin_LIBRARIES}
)
add_dependencies(
		${PROJECT_NAME}_mock
		${catkin_EXPORTED_TARGETS}
)
