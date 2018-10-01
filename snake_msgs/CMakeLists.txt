cmake_minimum_required(VERSION 2.8.3)
project(snake_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
  FILES
  Spline.msg
  Trajectory.msg
	Waypoint.msg
	Waypoints.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs    
)

catkin_package()
