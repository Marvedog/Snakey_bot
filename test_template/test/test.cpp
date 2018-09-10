#include <geometry_msgs/Twist.h>

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mutex>

namespace
{
	double refresh_rate = 0.5;
}

ros::NodeHandle* g_n=NULL;

class TestClient : public testing::Test 
{
	public:
		TestClient()
		{
			got_twist = false;
		}

		~TestClient() {}

		bool got_twist;

		geometry_msgs::Twist twist;

		inline void set_twist(bool val) {twist_m.lock(); got_twist = true; twist_m.unlock();}

		void 
		velCb(const boost::shared_ptr<geometry_msgs::Twist const>& msg)
		{
			twist = *msg;
			got_twist = true;
		}

    ::testing::AssertionResult
	  lessThan(double val1, double val2)
	  {
			if (val1 - val2 < 0)
				return ::testing::AssertionSuccess() << val1-val2 << " < " << 0;
			return ::testing::AssertionFailure() << val1-val2 << " > " << 0;
		}

	private:
		std::mutex twist_m;
};

TEST_F(TestClient, Testen)
{
	ros::Subscriber sub = g_n->subscribe<geometry_msgs::Twist>("pub/vel", 1, boost::bind(&TestClient::velCb, this, _1));

	int i = 10; set_twist(false);
	while (!got_twist && i > 0)
	{
		ros::spinOnce();
		ros::WallDuration d(refresh_rate);
		d.sleep();
		i--;
	}

	{
		SCOPED_TRACE("PUBTEST");
		if (got_twist)
		{
			EXPECT_TRUE(lessThan(twist.linear.x, 1.0));
		}
	}

	{
	SCOPED_TRACE("GOTMESSAGE");
		ASSERT_TRUE(got_twist);
	}
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "template_test");
	g_n = new ros::NodeHandle();

	/* Wait for sim time to begin */
	while (ros::Time::now().toSec() <= 0)
	{
		ros::Duration(0.25).toSec();
		ros::spinOnce();
	}

	ros::Time start_time = ros::Time::now();
  while ((ros::Time::now().toSec() - start_time.toSec()) < ros::Duration(3).toSec())
	{
		ros::Duration(0.25).sleep();
		ros::spinOnce();
	}

	int result = RUN_ALL_TESTS();

	delete g_n;
	return result;
}
