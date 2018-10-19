#ifndef SNAKE_VISUALIZATION_H
#define SNAKE_VISUALIZATION_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <snake_msgs/JointAngle.h>
#include <snake_msgs/JointAngles.h>
#include <visualization_msgs/Marker.h>

#include <pkg_utils/snake.h>
#include <vector>
#include <string>

/// TF deps
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

class SnakeViz
{
	public:
		SnakeViz( const ros::NodeHandle &nh
						, std::vector<std::string> snake_config
						, std::vector<double> d
						, std::vector<double> a
						, std::vector<double> alpha
						, std::vector<double> theta
						, int base);

	private:

		void jointCb(const snake_msgs::JointAngles joints);
		void odomCb(const nav_msgs::Odometry odom);

		void jointMsgToVector(const snake_msgs::JointAngles &snake_msg);
		void geometryPose3ToTf(const geometry::Pose3 &tf, tf::Transform &tf_ros) const;

		void pubMapBaseTransform(const nav_msgs::Odometry &odom);
		void pubTransform(const geometry::Pose3 &tf, const std::string &s);


		Snake snake; 
		std::vector<double> theta;
		std::vector<std::string> frames;

		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber jointsub;
		ros::Subscriber odomsub;

		visualization_msgs::Marker markerSetup();

		tf::TransformBroadcaster bc;
};

#endif
