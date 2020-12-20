/*
 * rto_local_move_server_node.cpp
 *
 *  Created on: 14.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RTOLocalMoveServer.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rto_local_move_server_node");
	RTOLocalMoveServer rlms;
	rlms.spin();

	return 0;
}
