#ifndef _HELPER_FUNCTIONS_
#define _HELPER_FUNCTIONS_

#include <cstddef>
#include <string>
#include "ros/ros.h"

double normalize_angle_plusmin_pi(double angle);
double saturate(double value, double min, double max);
int partialFactorial(int x, std::size_t n);

namespace helper_functions {
	// TODO: Include all functions inside this namespace and add namespace to all function call in various nodes

    template<typename type>
    bool getRosParamAndPrintRosWarningIfFail(const std::string param_name, type &ros_param, ros::NodeHandle* nh, const std::string package_name, type default_value ){
        bool gotParam = true;
        if (!nh->getParam(param_name, ros_param)){
            gotParam = false;
            ros_param = default_value;
            ROS_WARN_STREAM("PARAM NOT FOUND, for param: '" << param_name << "', in pkg: '" << package_name << "'. Using default value: '" << ros_param <<"'.");
        }
        return gotParam;
    }
}
#endif
//ros::NodeHandle* = &nh_