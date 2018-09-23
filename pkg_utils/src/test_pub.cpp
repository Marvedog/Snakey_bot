#include <ros/ros.h>
#include <ros/time.h>
#include <pkg_utils/Testmsg.h>

int 
main(int argc, char **argv)
{
	ros::init(argc, argv, "publish");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<pkg_utils::Testmsg>("pub/msg", 100);
	ros::Rate rate(20);

	while (ros::ok())
	{
		/* Construct timing message */
		pkg_utils::Testmsg msg;

		/* Start time */
		msg.start.data = ros::Time::now();
			
		/* Just to get some time in-between */
		rate.sleep();

		/* End time */
		msg.end.data = ros::Time::now();
			
		/* Publish msg */
		pub.publish(msg);

		rate.sleep();
	}
}
