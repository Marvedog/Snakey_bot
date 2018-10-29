cmake_minimum_required(VERSION 2.8.3)
project(snake_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  message_runtime
  std_msgs
	tf
)

add_message_files(
  FILES
	JointAngle.msg
	JointAngles.msg
	Shape.msg
	Spline.msg
  Trajectory.msg
	Transforms.msg
	Vlos.msg
	Waypoint.msg
	Waypoints.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
	 tf 
)

catkin_package()
