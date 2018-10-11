#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <snake_msgs/Vlos.h>

class GuidanceVisualization 
{
	public:
    GuidanceVisualization();
    void vlosCb(const snake_msgs::Vlos::ConstPtr msg);
    void odomRefCb(const nav_msgs::Odometry::ConstPtr msg);

	private:
    ros::NodeHandle nh;
    ros::Publisher vlospub;
    ros::Publisher odometrypub;
    ros::Subscriber vlossub;
    ros::Subscriber odometrysub;

		nav_msgs::Odometry odomref;

		/* Setup line marker to viz spline */
		visualization_msgs::Marker arrowMarkerSetup(const nav_msgs::Odometry &msg);
		visualization_msgs::Marker lineMarkerSetup();
};
