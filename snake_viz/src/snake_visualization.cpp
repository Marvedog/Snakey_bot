#include <snake_viz/snake_visualization.h>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

SnakeViz::SnakeViz( const ros::NodeHandle &nh
	 								,	std::vector<std::string> snake_config
		              , std::vector<double> d
									, std::vector<double> a
								  , std::vector<double> alpha
									, std::vector<double> theta
								  , int base)
: snake( snake_config
			 , d
	 		 , a
	 		 , alpha
			 , theta
			 , base )
, nh(nh)
{
	this->jointsub = this->nh.subscribe("joint_angles", 1, &SnakeViz::jointCb, this);
	this->odomsub = this->nh.subscribe("odometry", 1, &SnakeViz::odomCb, this);

	this->theta.resize(theta.size());
	this->frames.resize(theta.size());
}

void 
SnakeViz::jointCb(const snake_msgs::JointAngles joints) 
{
	this->jointMsgToVector(joints);	
	this->snake.computeJointTransformations(this->theta);

	for (int i = 0; i < this->snake.joint_tf.size(); i++)
	{
		this->pubTransform(this->snake.joint_tf[i], this->frames[i]);	
	}
}

void
SnakeViz::odomCb(const nav_msgs::Odometry odom)
{
	this->pubMapBaseTransform(odom);
}

void
SnakeViz::jointMsgToVector(const snake_msgs::JointAngles &snake_msg)
{
	for (int i = 0; i < snake_msg.theta.size(); i++)
	{
		this->theta[i] = snake_msg.theta[i].angle;
		this->frames[i] = snake_msg.theta[i].header.frame_id;
	}
}

/*
void 
SnakeViz::geometryPose3ToTf(const geometry::Pose3 &tf, tf::Transform &tf_ros) const 
{
	geometry_msgs::Quaternion quat;
	geometry_msgs::Pose pose;
	pose.position.x = tf.x();
	pose.position.y = tf.y();
	pose.position.z = tf.z();
	tf::quaternionEigenToMsg(tf.rot(), quat);
	pose.orientation = quat;

	tf::poseMsgToTF(pose, tf_ros);
}
*/

void
SnakeViz::pubMapBaseTransform(const nav_msgs::Odometry &odom)
{
	tf::Transform tf_odom;
	tf::poseMsgToTF(odom.pose.pose, tf_odom);
	
	this->bc.sendTransform( tf::StampedTransform( tf_odom
																							, ros::Time::now()
																							, "map"
																							, "base_link")
	);
}

void
SnakeViz::pubTransform(const tf::Transform &tf, const std::string &s)
{
	//this->geometryPose3ToTf(tf, tf_ros);
	bc.sendTransform( tf::StampedTransform( tf
																				, ros::Time::now()
																				, "base_link"
																				, s)
	);
}

visualization_msgs::Marker 
SnakeViz::markerSetup() 
{
  visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;
	
	marker.scale.x = 0.1;

	return marker;
}

int 
main( int argc, char** argv ) 
{
	ros::init(argc, argv, "racetrack_visualization");
	ros::NodeHandle nh;

	std::vector<std::string> snake_config;
	nh.getParam("snake_config", snake_config);

	std::vector<double> d;
	nh.getParam("d", d);

	std::vector<double> a;
	nh.getParam("a", a);

	std::vector<double> alpha;
	nh.getParam("alpha", alpha);

	std::vector<double> theta;
	nh.getParam("theta", theta);
	
	int base;
	nh.getParam("base", base);
	
	SnakeViz snakeviz(nh, snake_config, d, a, alpha, theta, base);
  
	//geometry::Pose3 pos = geometry::DHTransform(1,1,3.14,0).pose;
	
	//geometry::Pose3 pos1 = geometry::DHTransform(1,1,3.14/4,3.14/4).pose;

	//geometry::Pose3 pos2 = geometry::DHTransform(1,1,0,3.14).pose;
	ros::spin();
}
