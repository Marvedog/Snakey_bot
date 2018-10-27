#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tfMessage.h>
#include <nav_msgs/Odometry.h>
#include <snake_msgs/Transforms.h>

#include <pkg_utils/eelume_definitions.h>

#include <vector>
#include <string>

struct EigenTf
{
	Eigen::Vector3d vec;
	Eigen::Matrix3d mat;
	std::string frame_id;
	std::string child_id;
	std::string type;
};

class IK
{
	public:
		IK(const ros::NodeHandle &nh);

	private:
		
		/// Ros stuff
		ros::NodeHandle nh;
		ros::Subscriber odomsub;
		ros::Subscriber kinematicssub;

		/// Geometric Jacobian for two directions
		/**
			* Assumes ordered input list of transformations
			*/
		void computeGeometricJacobians();
		void computeAdjoint(const Eigen::Matrix3d &rot, const Eigen::Vector3d &vec, Eigen::Matrix<double, 6,6> &ad);
		void computeAdjointInverse(const Eigen::Matrix3d &rot, const Eigen::Vector3d &vec, Eigen::Matrix<double, 6,6> &ad);
		Eigen::Matrix3d skew3d(const Eigen::Vector3d &p);

		Eigen::Matrix3d tfMatrix3x3ToEigen(const tf::Matrix3x3 &mat);
		Eigen::Matrix3d tfvector3ToEigenSkew(const tf::Vector3 &vec);

		/// callbacks
		void odomCb(const nav_msgs::Odometry &odom);
		void kinematicsCb(const snake_msgs::Transforms &tf);
		void timerCb(const ros::TimerEvent &e);

		int joints;
		int base_frame;
		int dynamic_frames_front;
		int dynamic_frames_rear;

		/// Ros utils
		ros::Timer timer;
		double tstimer;

		/// Transformation storage
		std::vector<EigenTf> T_base;
		std::vector<EigenTf> T_home;
		
		/// Geometric jacobians
		Eigen::MatrixXd J_g_f;
		Eigen::MatrixXd J_g_r;

		/// Utilities
		Eigen::VectorXd X_z; /// Body twist z

		nav_msgs::Odometry odom;

		ros::Time odomstamp;
		ros::Time kinematicsstamp;

		bool debug;
};


IK::IK(const ros::NodeHandle &nh)
: nh(nh)
, base_frame(-1)
{
	std::string odom_topic, kinematics_topic;
	this->nh.getParam("/odom_topic", odom_topic);
	this->nh.getParam("/kinematics_topic", kinematics_topic);

	this->odomsub = this->nh.subscribe(odom_topic, 1, &IK::odomCb, this);	
	this->kinematicssub = this->nh.subscribe(kinematics_topic, 1, &IK::kinematicsCb, this);	
	
	while (this->base_frame == -1)
		ros::spinOnce();
	ROS_INFO_STREAM("IK:: got first kinematics message");

	this->nh.getParam("inverse_kinematics_ts", this->tstimer);
	this->timer = this->nh.createTimer(ros::Duration(this->tstimer), &IK::timerCb, this);
	
	this->nh.getParam("inverse_kinematics_debug", this->debug);
	if (this->debug)
		std::cout << "Debug mode on" << std::endl;
}

void 
IK::odomCb(const nav_msgs::Odometry &odom)
{
	this->odom = odom;
	this->odomstamp = ros::Time::now();
}
		
void 
IK::kinematicsCb(const snake_msgs::Transforms &tf)
{
	this->joints = tf.joints;
	this->base_frame = tf.base_frame;
	for (int i = 0; i < this->joints; i++)
	{
		/// Geometry msg to tf
		tf::Transform _base, _home;
		tf::transformMsgToTF(tf.T_home.transforms[i].transform, _home);
		tf::transformMsgToTF(tf.T_base.transforms[i].transform, _base);

		/// Tf to eigen
		tf::vectorTFToEigen(_home.getOrigin(), this->T_home[i].vec);
		tf::vectorTFToEigen(_base.getOrigin(), this->T_base[i].vec);
		tf::matrixTFToEigen(_home.getBasis(), this->T_home[i].mat);
		tf::matrixTFToEigen(_base.getBasis(), this->T_base[i].mat);

		/// Store frame id's
		this->T_home[i].frame_id = tf.T_home.transforms[i].header.frame_id;
		this->T_base[i].frame_id = tf.T_base.transforms[i].header.frame_id;
		this->T_home[i].child_id = tf.T_home.transforms[i].child_frame_id;
		this->T_base[i].child_id = tf.T_base.transforms[i].child_frame_id;

		/// Store type
		this->T_home[i].type = tf.type[i];
		this->T_base[i].type = tf.type[i];

		this->dynamic_frames_front = tf.dynamic_frames_front;
		this->dynamic_frames_rear = tf.dynamic_frames_rear;
	}

	this->kinematicsstamp = ros::Time::now();
}

void 
IK::timerCb(const ros::TimerEvent &e)
{
	double tin = ros::Time::now().toSec();
	
	/// Geometry
	this->computeGeometricJacobians();
	
	/// TODO: Task handling


	double tout = ros::Time::now().toSec();
	ROS_INFO_STREAM("IK:: Profiling period is:: " << e.profile.last_duration);
	ROS_INFO_STREAM("IK:: Runtime:: " << tout-tin);
}

void
IK::computeGeometricJacobians()
{
	///-----------------------------------------------------
	/// Front geometric jacobian
	///-----------------------------------------------------
	
	/// Jacobian from base link to front ee
	Eigen::MatrixXd J_I_f = Eigen::MatrixXd::Zero(6, this->dynamic_frames_front-1); 

	/// Loop over home transformations
	int j = 0;
	for (int i = this->base_frame; i > 0; i--)
	{

		/// Add to 
		if (this->T_home[i].type == eelume::joint)
		{
			Eigen::Matrix<double, 6, 6> ad;
			this->computeAdjoint(this->T_home[i].mat, this->T_home[i].vec, ad);
			J_I_f.block(j,0,6,1) = 	ad*this->X_z;
		}

	}

	if (this->debug)
	{
		std::cout << "IK:: Manipulator Jacobian front" << std::endl;
		std::cout << J_I_f << std::endl;
	}

	Eigen::Matrix<double, 6, 6> ad_inv_f;
	this->computeAdjointInverse(this->T_home[0].mat, this->T_home[0].vec, ad_inv_f);
	this->J_g_f << ad_inv_f, ad_inv_f*J_I_f;
	
	if (this->debug)
	{
		std::cout << "IK:: Geometric Jacobian front" << std::endl;
		std::cout << J_g_f << std::endl;
	}

	///-----------------------------------------------------
	/// Rear geometric jacobian
	///-----------------------------------------------------
	
	/// Jacobian from base link to rear ee
	Eigen::MatrixXd J_I_r = Eigen::MatrixXd::Zero(6, this->dynamic_frames_rear-1);
	
	/// Loop over home transformations
	j = 0;
	for (int i = this->base_frame; i < this->joints-1; i++)
	{
		/// Add to 
		if (this->T_home[i].type == eelume::joint)
		{
			Eigen::Matrix<double, 6, 6> ad;
			this->computeAdjoint(this->T_home[i].mat, this->T_home[i].vec, ad);
			J_I_r.block(j,0,6,1) = 	ad*this->X_z;
		}
	}
	
	if (this->debug)
	{
		std::cout << "IK:: Manipulator Jacobian rear" << std::endl;
		std::cout << J_I_f << std::endl;
	}
	
	Eigen::Matrix<double, 6, 6> ad_inv_r;
	this->computeAdjointInverse(this->T_home[this->joints-1].mat, this->T_home[this->joints-1].vec, ad_inv_r);
	this->J_g_r << ad_inv_r, ad_inv_r*J_I_f;
	
	if (this->debug)
	{
		std::cout << "IK:: Geometric Jacobian rear" << std::endl;
		std::cout << J_g_r << std::endl;
	}
}


///-----------------------------------------------------
/// Utilities for computing geometric Jacobian
///-----------------------------------------------------

void
IK::computeAdjoint(const Eigen::Matrix3d &rot, const Eigen::Vector3d &vec, Eigen::Matrix<double, 6,6> &ad)
{
	Eigen::Matrix3d skew = this->skew3d(vec);
	ad << rot, skew*rot,
				Eigen::Matrix3d::Zero(), rot;
}

void
IK::computeAdjointInverse(const Eigen::Matrix3d &rot, const Eigen::Vector3d &vec, Eigen::Matrix<double, 6,6> &ad)
{
	Eigen::Matrix3d skew = this->skew3d(vec);
	ad << rot.transpose(), -rot.transpose()*skew,
		    Eigen::Matrix3d::Zero(), rot.transpose();
}

Eigen::Matrix3d
IK::skew3d(const Eigen::Vector3d &p)
{
	Eigen::Matrix3d skew;
	skew <<   0, -p(2),  p(1),
				 p(2),	   0, -p(0),
				-p(1),  p(0),     0;
	return skew;
}

int 
main(int argc, char **argv) 
{
	ros::init(argc, argv, "inverse_kinematics_node");
	ros::NodeHandle nh;
	
	IK ik(nh);
	ros::spin();
	return 0;
}
