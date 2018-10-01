#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <snake_msgs/Spline.h>

class PathVisualization 
{
	public:
    PathVisualization();
    void pathCb(const snake_msgs::Spline::ConstPtr msg);

	private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

		/* Setup line marker to viz spline */
		visualization_msgs::Marker markerSetup();
};
