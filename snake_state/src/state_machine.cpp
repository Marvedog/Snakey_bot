#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <snake_msgs/JointAngles.h>

#include <pkg_utils/snake.h>
#include <pkg_utils/eelume_definitions.h>

#include <vector>
#include <string>

class State
{
	public:
		State( const ros::NodeHandle &nh
				 , std::vector<std::string> snake_config
				 , std::vector<double> d
				 , std::vector<double> a 
				 , std::vector<double> alpha
				 , std::vector<double> theta
				 , int base_frame);

	private:

		ros::NodeHandle nh;
		ros::Subscriber jointsub;
		ros::Subscriber odomsub;
		ros::Publisher transformpub;

		void jointCb(const snake_msgs::JointAngles joints);
		void odomCb(const nav_msgs::Odometry odom);
		void publishFrames();
		void publishTransform(const eelume::Tf &tf);

		/// Only necessary until slam works!
		void publishMapBaseTransform(const nav_msgs::Odometry &odom);
		
		int base_frame;

		std::vector<double> theta;
		std::vector<eelume::Tf> T_joint;
		std::vector<eelume::Tf> T_b_i;
		std::vector<eelume::Tf> T_b_i_h;

		tf::TransformBroadcaster bc;

		/// Prints the transformations T_joint, T_b_i and T_b_i_h
		bool visualization;
		void visualizeTransforms();

		Snake snake;

		void jointMsgToVector(const snake_msgs::JointAngles &snake_msg);
		geometry_msgs::TransformStamped tfStructToStampedTransform(const eelume::Tf &tf);
};

State::State(  const ros::NodeHandle &nh
						 , std::vector<std::string> snake_config
						 , std::vector<double> d
						 , std::vector<double> a 
						 , std::vector<double> alpha
						 , std::vector<double> theta
						 , int base_frame)
: snake( snake_config
		   , d				
			 , a
			 , alpha
			 , theta
			 , base_frame )
, nh(nh)
, base_frame(base_frame)
{
	this->nh.getParam("visualization", this->visualization);

	std::string joint_topic; 
	this->nh.getParam("joint_topic", joint_topic);
	
	std::string odom_topic; 
	this->nh.getParam("odom_topic", odom_topic);

	std::string transform_topic;
	this->nh.getParam("transform_topic", transform_topic);

	this->jointsub = this->nh.subscribe(joint_topic, 1, &State::jointCb, this);
	this->odomsub = this->nh.subscribe(odom_topic, 1, &State::odomCb, this);
	this->transformpub = this->nh.advertise<tf::tfMessage>(transform_topic, 1);

	this->theta.resize(this->snake.getJoints());
	this->T_joint.resize(this->snake.getJoints());
	this->T_b_i.resize(this->snake.getJoints());
	this->T_b_i_h.resize(this->snake.getJoints());

	ROS_INFO_STREAM("---------------- State machine construction completed ------------");
}

void
State::jointCb(const snake_msgs::JointAngles joints)
{
	this->jointMsgToVector(joints);
	
	this->snake.updateTransformations(this->theta);
	this->snake.getFrames(this->T_joint, this->T_b_i, this->T_b_i_h);
	
	this->publishFrames();

	if (this->visualization)
		this->visualizeTransforms();
}

void
State::odomCb(const nav_msgs::Odometry odom)
{
	if (this->visualization)
		this->publishMapBaseTransform(odom);
}

void
State::jointMsgToVector(const snake_msgs::JointAngles &snake_msg)
{
	for (int i = 0; i < snake_msg.theta.size(); i++)
	{
		this->theta[i] = snake_msg.theta[i].angle;
	}
}

void
State::publishMapBaseTransform(const nav_msgs::Odometry &odom)
{
	tf::Transform tf_odom;
	tf::poseMsgToTF(odom.pose.pose, tf_odom);
	bc.sendTransform( tf::StampedTransform( tf_odom
																							, ros::Time::now()
																							, "map"
																							, "base_link")
	);
}

void
State::publishTransform(const eelume::Tf &tf)
{
	bc.sendTransform( tf::StampedTransform( tf.tf
																				, ros::Time::now()
																				, tf.frame_id
																				, tf.child_id)
	);
}

void
State::publishFrames()
{
	tf::tfMessage _Tf;
	for (int i = 0; i < this->snake.getJoints(); i++)
	{
		ROS_ERROR_STREAM(i);
		if (i != this->snake.getJoints()-1)
			_Tf.transforms.push_back(tfStructToStampedTransform(this->T_joint[i]));
		_Tf.transforms.push_back(tfStructToStampedTransform(this->T_b_i[i]));
		_Tf.transforms.push_back(tfStructToStampedTransform(this->T_b_i_h[i]));
	}
	this->transformpub.publish(_Tf);
}

geometry_msgs::TransformStamped
State::tfStructToStampedTransform(const eelume::Tf &tf)
{
	geometry_msgs::TransformStamped _tf;
	_tf.header.frame_id = tf.frame_id;
	_tf.child_frame_id = tf.child_id;
 	tf::transformTFToMsg(tf.tf, _tf.transform);
	return _tf;	
}

void
State::visualizeTransforms()
{
	for (int i = 0; i < this->T_joint.size(); i++)
	{
		if (i < this->T_joint.size()-1)
			this->publishTransform(this->T_joint[i]);
		this->publishTransform(this->T_b_i[i]);
		this->publishTransform(this->T_b_i_h[i]);
	}
}

int 
main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");
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
	
	int base_frame;
	nh.getParam("base_frame", base_frame);
	
	State state(nh, snake_config, d, a, alpha, theta, base_frame);
  
	ros::spin();

	return 0;
}
