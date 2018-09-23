cmake_minimum_required(VERSION 2.8.3)
project(pkg_utils)

add_compile_options(-std=c++11)

set(Eigen3_DIR /usr/lib/cmake/eigen3)
set(GTEST_DIR /usr/src/gtest/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
	std_msgs
	message_generation
	rostest
	message_runtime
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
  LIBRARIES test_base test_statistics
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

# Create statistics library for testing
add_library(testing_statistics src/testing_statistics.cpp include/pkg_utils/testing_statistics.h)
target_link_libraries(
	testing_statistics
	${catkin_LIBRARIES}
)

add_dependencies(
	testing_statistics	
	${catkin_EXPORTED_TARGETS}
)

# Executable for verification of test and illustration
add_executable(
	test_pub 
	src/test_pub.cpp
)

target_link_libraries(
	test_pub 
	${catkin_LIBRARIES}
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
		add_dependencies(tests bare_bones_test)
	endif()

	set(LOCAL_DEPENDENCIES
		bare_bones_test
		src/test_pub.cpp
	)

	add_rostest(test/test_template_bare_bones.test DEPENDENCIES ${LOCAL_DEPENDENCIES})
endif()
