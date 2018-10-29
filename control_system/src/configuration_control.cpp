#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <snake_msgs/Vlos.h>
#include <snake_msgs/Shape.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

class ShapeControl
{
	public:
		ShapeControl(const ros::NodeHandle &nh);

	private: 
		ros::NodeHandle nh;
		ros::Subscriber odomsub;
		ros::Subscriber vlossub;
		ros::Publisher shapepub;

		void odomCb(const nav_msgs::Odometry odom);
		void vlosCb(const snake_msgs::Vlos vlos);

		int base_frame;
		int frames;
		double cf;
		double lf;
		double lr;

		tf::Vector3 vlos;
		tf::Vector3 xf;
		tf::Vector3 xr;
		tf::Transform T_i_b;

		tf::TransformBroadcaster bc;
};

ShapeControl::ShapeControl(const ros::NodeHandle &nh)
: nh(nh)
{
	this->odomsub = this->nh.subscribe("/odometry", 1, &ShapeControl::odomCb, this);
	this->vlossub = this->nh.subscribe("vlos", 1, &ShapeControl::vlosCb, this);
	this->shapepub = this->nh.advertise<snake_msgs::Shape>("shape", 1);
	
	/// Configure front and rear idle length
	std::vector <double> l;
	this->nh.getParam("/a", l);
	this->nh.getParam("/base_frame", this->base_frame);
	this->frames = l.size();

	this->lf = 0;	
	this->xf.setZero();
	for (int i = this->base_frame-1; i > 0; i--)
		this->lf += l[i];
	this->xf[0] = -lf;

	this->lr = 0;
	this->xr.setZero();
	for (int i = this->base_frame; i < l.size(); i++)
		this->lr += l[i];
	this->xr[0] = lr;

	/// Get shape factor
	this->nh.getParam("cf", cf);

	this->vlos.setZero();	
}

void
ShapeControl::odomCb(const nav_msgs::Odometry odom)
{
	geometry_msgs::Transform tf_g;
	tf_g.translation.x = 0;
	tf_g.translation.y = 0;
	tf_g.translation.z = 0;
	tf_g.rotation = odom.pose.pose.orientation;
	tf::transformMsgToTF(tf_g, this->T_i_b);
}

void
ShapeControl::vlosCb(const snake_msgs::Vlos vlos)
{
	/// Store vlos vector
	this->vlos[0] = vlos.x1-vlos.x0;
	this->vlos[1] = vlos.y1-vlos.y0;
	this->vlos[2] = vlos.z1-vlos.z0;

	this->vlos = this->T_i_b * this->vlos;

	tf::Vector3 xf_I, xr_I;
	if (this->base_frame > 2)
	{
		/// Compute rotation angle between front and vlos
		double gamma_f = this->xf.angle(this->vlos);
		std::cout << "Gamma " << cf*gamma_f << std::endl;

		xf_I = this->xf;
		
		tf::Vector3 k = xf_I.cross(this->vlos);

		/// Rotation
		tf::Quaternion q_f(k, cf*gamma_f); tf::Transform _tf_f(q_f);
		
		/// Rotate with shape angle
		xf_I = _tf_f * xf_I;
	
		tf::Transform tf_f; tf_f.setIdentity(); tf_f.setOrigin(xf_I);
		bc.sendTransform(tf::StampedTransform(tf_f, ros::Time::now(), "base_link", "p_front_ref"));
	}

	if (this->base_frame < this->frames-2)	
	{
		tf::Vector3 xr_rev = -this->xr;
		double gamma_r = xr_rev.angle(this->vlos);
		tf::Vector3 k = xr_rev.cross(this->vlos);
		
		xr_I = this->xr;
		
		/// Rotation
		tf::Quaternion q_r(k, -cf*gamma_r); tf::Transform _tf_r(q_r);

		/// Rotate with shape angle
		xr_I = _tf_r * xr_I;
	
		/// Publish transform for visualization
		tf::Transform tf_r; tf_r.setIdentity(); tf_r.setOrigin(xr_I);
		bc.sendTransform(tf::StampedTransform(tf_r, ros::Time::now(), "base_link", "p_rear_ref"));
	}

	/// Publish desired end_effector poses
	snake_msgs::Shape shape;
	shape.xf = vlos.x0 + xf_I[0];
	shape.yf = vlos.y0 + xf_I[1];
	shape.zf = vlos.z0 + xf_I[2];
	shape.xr = vlos.x0 + xr_I[0];
	shape.yr = vlos.y0 + xr_I[1];
	shape.zr = vlos.z0 + xr_I[2];
	shapepub.publish(shape);

}

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "shape_controller_node");
	ros::NodeHandle nh;

	ShapeControl shapecontrol(nh);
	ros::spin();
}
