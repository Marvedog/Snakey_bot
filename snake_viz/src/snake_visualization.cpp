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
, base(base)
{
	this->jointsub = this->nh.subscribe("joint_angles", 1, &SnakeViz::jointCb, this);
	this->odomsub = this->nh.subscribe("odometry", 1, &SnakeViz::odomCb, this);

	this->theta.resize(this->snake.getJoints());
	this->T_joint.resize(this->theta.size());
	this->T_b_i.resize(this->theta.size());
	this->T_b_i_h.resize(this->theta.size());

	ROS_INFO_STREAM("---------- SNAKE VISUALIZATION:: Constructor initialized ----------");
}

void 
SnakeViz::jointCb(const snake_msgs::JointAngles joints) 
{
	this->jointMsgToVector(joints);	
	this->snake.updateTransformations(this->theta);
	
	this->snake.getFrames(this->T_joint, this->T_b_i, this->T_b_i_h);

	for (int i = 0; i < this->T_joint.size()-1; i++)
	{
		//this->pubTransform(this->T_joint[i]);
		//this->pubTransform(this->T_b_i[i]);
		this->pubTransform(this->T_b_i_h[i]);
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
	}
}

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
SnakeViz::pubTransform(const eelume::Tf &tf)
{
	bc.sendTransform( tf::StampedTransform( tf.tf
																				, ros::Time::now()
																				, tf.frame_id
																				, tf.child_id)
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
  
	ros::spin();
}
