/*
 * rto_local_move_client_node.cpp
 *
 *  Created on: 14.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RTOLocalMoveClient.h"

#define PI 3.14159

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rto_local_move_client_node");
	RTOLocalMoveClient rlmc;
	rto_local_move::LocalMoveGoal goal;
	float max_time;

	if( argc > 1 )
	{
		std::istringstream is( argv[1] );
		is >> goal.move_x;
	}
	else
	{
		ROS_WARN( "Possibly too few arguments. Usage: \"rosrun rto_local_move rto_local_move_client_node move_x[m] [move_y[m] [rotate[deg]] max_time[s] ]\"." );
	}

	if( argc > 2 )
	{
		std::istringstream is( argv[2] );
		is >> goal.move_y;
	}

	if( argc > 3 )
	{
		float value;
		std::istringstream is( argv[3] );
		is >> value;
		goal.rotation = ( value * PI ) / 180; // To radians
		goal.ignore_rotation = false;
	}

	if( argc > 4 )
	{
		std::istringstream is( argv[4] );
		is >> max_time;
		rlmc.setMaxTime( max_time );
	}

	std::ostringstream os;
	if( goal.ignore_rotation )
	{
		os << "na";
	}
	else
	{
		os << ( goal.rotation * 180 ) / PI; // To degrees
	}

	ROS_INFO( "Sending goal (move_x[m], move_y[m], rotate[deg] max_time[s])=(%f, %f, %s, %f)",
			goal.move_x, goal.move_y, os.str().c_str(), max_time );

	if( rlmc.checkServer() )
	{
		rlmc.sendGoal( goal );
		rlmc.spin();
	}

	return 0;
}
