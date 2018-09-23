#include "pkg_utils/testing_base_class.h"
#include "pkg_utils/testing_statistics.h"
#include "pkg_utils/Testmsg.h"

#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/service.h>
#include <ros/console.h>
#include <ros/package.h>

#include <random>
#include <algorithm>
#include <string>

using namespace Test;

/* ---------- Global nodehandle ------------ 
	 Why?
	 ----> This have to be accessible both in main
	 ----> and within the test. Thus it is defined
	 ----> globally 
*/
ros::NodeHandle* g_n=NULL;

/* ---------- Global testing variables -----------
	 Why? 
	 -----> We need to read parameters from a parameter
	 -----> file (they are read in the main function)
	 -----> and pass them to the test class. 
*/

namespace
{
	/* How many number of runs left until the test is finished */
	int number_of_runs_left;

	/* The base name of the file we want to store the test data to */
  std::string log_file_base;

	/* Maximum latency for the node under test */
	double max_latency = 0.05;
		
	/* Floating point tolerance */
	double floating_tol;

	/* How often we refresh while we wait for a new message */
  double refresh_rate;

	/* How many callbacks we have in our test class */
	const int number_callbacks = 1;

	/* --------- Testing class ---------
	   Why? 
		 -----> The testing class is passed to an argument 
		 -----> to each of the tests in this program. It
		 -----> provides utilities to read messages from
		 -----> nodes and define utilities to make the 
		 -----> test output readable and practical for our 
		 -----> purpose. 
	 */
	class TestClient 
	: public TestingBase<number_callbacks>  /* Synchronization */
	, public Statistics 										/* For post-analysis */
	, public testing::Test  											/* Google test utilities */
	{
		public:
			TestClient()
			{;}

			~TestClient() {}

			/* Container for receiving messages */
    	pkg_utils::Testmsg msg_;

	  	/* Callback for waypoint messages */
			void msgCb(const boost::shared_ptr<pkg_utils::Testmsg const>& msg);

			/* Array for handling post processing statistics */
			std::vector<double> timing_data;
    
			/* Return true if val1 < val2. For printing assertions to screen */
			::testing::AssertionResult
	  	lessThan(double val1, double val2)
	  	{
				if (val1 - val2 < 0)
					return ::testing::AssertionSuccess() << val1-val2 << " < " << 0;
				return ::testing::AssertionFailure() << val1-val2 << " > " << 0;
			}
	};
		
	void
	TestClient::msgCb(const boost::shared_ptr<pkg_utils::Testmsg const>& msg)
	{
		msg_ = *msg; this->setNewMessageReceived(true, 0);
	}



TEST_F(TestClient, bare_bones_test)
{
	
	/* Define subscriber for receiving incoming messages */
	ros::Subscriber msg_sub = g_n->subscribe<pkg_utils::Testmsg>("/test/pub/msg", 1, boost::bind(&TestClient::msgCb, this, _1));

	/* Sample timing data number_of_runs_left times more */
	while (number_of_runs_left > 0)
	{
		
		/* Refresh data */
		int i = 10; this->setNewMessageReceived(false, 0);
		
		/* Actively wait for a new message to arrive */
		while ((!this->getNewMessageReceived(0)) && i > 0 )
		{
			ros::spinOnce();
			ros::WallDuration d(refresh_rate);
			d.sleep();
			i--;
		}

		/* Local scope to localize where a test may fail.
			 This is a great utility when the tests grow bigger.
		 */
		{
		SCOPED_TRACE("MSG_TIMING_VERIFICATION");
			
			/* Just double checking */
			if (this->getNewMessageReceived(0))
			{
				/* Extract timing data */
				ros::Duration latency = (msg_.end.data - msg_.start.data);	
				
				/* Verify if we exceed our maximum timing limit */
				EXPECT_TRUE(this->lessThan(latency.toSec(), max_latency));

				/* Store timing data */
				timing_data.push_back(latency.toSec());
			}
		}
		number_of_runs_left--;

	} /* End SCOPED_TRACE */

	/* Preallocation of timing statistics */
  double mean_time;
	double var_time;
	double med_time;
  double min_time;
  double max_time;
	
	/* Logging of timing statistics */
  this->computeStatistics(
    timing_data, log_file_base, "msg_runtime", mean_time,
    var_time, med_time, min_time, max_time);
}
} /* End namespace TestExample */ 

int
main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

	ros::init(argc, argv, "bare_bones_test");
	g_n = new ros::NodeHandle();

	/* Wait for sim time to begin */
	while (ros::Time::now().toSec() <= 0)
	{
		ros::Duration(0.25).toSec();
		ros::spinOnce();
	}

	/* Process parameters */
	if (!g_n->getParam("test/number_of_runs", number_of_runs_left))
	{
		ROS_ERROR_STREAM("Missing parameter: test/number_of_runs");
		delete g_n;
		return 0;
	}
	
	if (!g_n->getParam("test/log_file_base", log_file_base))
	{
		ROS_ERROR_STREAM("Missing parameter: test/log_file_base");
		delete g_n;
		return 0;
	}

	if (!g_n->getParam("test/max_latency", max_latency))
	{
		ROS_ERROR_STREAM("Missing parameter: test/max_latency");
		delete g_n;
		return 0;
	}
	
	if (!g_n->getParam("test/floating_tol", floating_tol))
	{
		ROS_ERROR_STREAM("Missing parameter: test/floating_tol");
		delete g_n;
		return 0;
	}

	if (!g_n->getParam("test/refresh_rate", refresh_rate))
	{
		ROS_ERROR_STREAM("Missing parameter: test/refresh_rate");
		delete g_n;
		return 0;
	}
	/*
	if (!g_n->getParam("test/number_callbacks", number_callbacks))
	{
		ROS_ERROR_STREAM("Missing parameter: test/number_callbacks");
		delete g_n;
		return 0;
	}
	*/

	float del;	
	if (!g_n->getParam("test/delay", del))
	{
		ROS_ERROR_STREAM("Missing parameter: test/delay");
		delete g_n;
		return 0;
	}
	ros::Duration delay = ros::Duration(del);

	/* Give nodes time to start up */
	ros::Time start_time = ros::Time::now();
	while ((ros::Time::now().toSec() - start_time.toSec()) < delay.toSec())
	{
		ros::Duration(0.25).sleep();
		ros::spinOnce();
	}

  int result = RUN_ALL_TESTS();

	delete g_n;
	return result;
}
