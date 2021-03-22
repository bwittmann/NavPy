/*
 * rto_safety_node.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#include "RTOSafety.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rto_safety_node");

	RTOSafety rs;
	rs.spin();

	return 0;
}
