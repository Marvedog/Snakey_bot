cmake_minimum_required(VERSION 2.8.3)
project(pkg_utils)

add_compile_options(-std=c++11)

set(Eigen3_DIR /usr/lib/cmake/eigen3)
set(GTEST_DIR /usr/src/gtest/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  Eigen3
	std_msgs
	message_generation
	rostest
	message_runtime
	eigen_conversions
)

# Rostest utilities, multithreading. 
find_package(
	Boost
	REQUIRED
	signals
)

add_message_files(
	FILES
	Testmsg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES testing_statistics helper_functions spline trajectory track eigen_helpers jacobian snake
)

include_directories(
  ${Eigen3_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	include
  /usr/include/eigen3
	/usr/include/gtest
)

# Helper function library
add_library(
	helper_functions 
	src/helper_functions.cpp 
	include/pkg_utils/helper_functions.h
)

# Eigen helper library
add_library(
	eigen_helpers 
	src/eigen_helpers.cpp 
	include/pkg_utils/eigen_helpers.h
)
target_link_libraries(
	eigen_helpers 
	${catkin_LIBRARIES}
)
add_dependencies(
	eigen_helpers 
	${catkin_EXPORTED_TARGETS}
)

# Create spline library	
add_library(
	spline 
	src/spline.cpp 
	src/eigen_helpers.cpp 
	src/helper_functions.cpp 
	include/pkg_utils/spline.h 
)
target_link_libraries(
		spline 
		${catkin_LIBRARIES}
)
add_dependencies(
		spline 
		${catkin_EXPORTED_TARGETS}
)

# Create statistics library for testing
add_library(
	testing_statistics 
	src/testing_statistics.cpp 
	include/pkg_utils/testing_statistics.h
)
target_link_libraries(
	testing_statistics
	${catkin_LIBRARIES}
)
add_dependencies(
	testing_statistics	
	${catkin_EXPORTED_TARGETS}
)

# Create trajectory library
add_library(
	track
	src/track.cpp 
	src/spline.cpp 
	src/eigen_helpers.cpp 
	include/pkg_utils/track.h
)
target_link_libraries(
	track
	${catkin_LIBRARIES}
)
add_dependencies(
	track
	${catkin_EXPORTED_TARGETS}
)

# Create trajectory library
add_library(
	trajectory
	src/trajectory.cpp 
	include/pkg_utils/trajectory.h
)
target_link_libraries(
	trajectory
	${catkin_LIBRARIES}
)
add_dependencies(
	trajectory	
	${catkin_EXPORTED_TARGETS}
)

# Create trajectory library
add_library(
	jacobian
	src/jacobian.cpp 
	include/pkg_utils/jacobian.h
)
target_link_libraries(
	jacobian
	${catkin_LIBRARIES}
)
add_dependencies(
	jacobian	
	${catkin_EXPORTED_TARGETS}
)

## Create trajectory library
#add_library(
#	modules
#	src/modules.cpp 
#	src/Pose3.cpp
#	include/pkg_utils/modules.h
#	include/pkg_utils/Pose3.h
#)
#target_link_libraries(
#	modules
#	${catkin_LIBRARIES}
#)
#add_dependencies(
#	modules	
#	${catkin_EXPORTED_TARGETS}
#)
# Create trajectory library
add_library(
	snake
	src/snake.cpp 
	include/pkg_utils/snake.h
)
target_link_libraries(
	snake
	${catkin_LIBRARIES}
)
add_dependencies(
	snake
	${catkin_EXPORTED_TARGETS}
)

# Executable for verification of test and illustration
# Executable for verification of test and illustration
add_executable(
	test_pub 
	src/test_pub.cpp
)

target_link_libraries(
	test_pub 
	${catkin_LIBRARIES}
)

# Test for eigen library
catkin_add_gtest(eigen_helpers_test src/eigen_helpers.cpp test/eigen_helpers_test.cpp)
target_link_libraries(eigen_helpers_test ${catkin_LIBRARIES})
add_dependencies(eigen_helpers_test ${catkin_EXPORTED_TARGETS})
	
# Test for transformations library
catkin_add_gtest(
	transformations_test 
	test/transformations_test.cpp
	include/pkg_utils/transformations.h 
)
target_link_libraries(
	transformations_test 
	${catkin_LIBRARIES}
)
add_dependencies(
	transformations_test 
	${catkin_EXPORTED_TARGETS}
)

# Test for spline library
catkin_add_gtest(
	spline_test 
	src/eigen_helpers.cpp 
	src/helper_functions.cpp 
	src/spline.cpp 
	test/spline_test.cpp
)
target_link_libraries(
	spline_test 
	${catkin_LIBRARIES}
)
add_dependencies(
	spline_test 
	${catkin_EXPORTED_TARGETS}
)

if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	if(TARGET tests)
		add_executable(
			bare_bones_test 
			EXCLUDE_FROM_ALL 
			test/test_template_bare_bones.cpp
			include/pkg_utils/testing_base_class.h)
		
		target_link_libraries(
			bare_bones_test 
			${catkin_LIBRARIES} 
			${GTEST_LIBRARIES}
			testing_statistics
		)
		add_dependencies(
			tests 
			bare_bones_test
		)
	endif()

	set(LOCAL_DEPENDENCIES
		bare_bones_test
		src/test_pub.cpp
	)

	add_rostest(test/test_template_bare_bones.test DEPENDENCIES ${LOCAL_DEPENDENCIES})
endif()
