/*
 * RTOLocalMoveServer.h
 *
 *  Created on: 13.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOLOCALMOVESERVER_H_
#define ROBOTINOLOCALMOVESERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

#include "rto_local_move/LocalMoveAction.h"
#include "rto_local_move/LocalMoveActionGoal.h"

#define PI 3.14159

typedef actionlib::SimpleActionServer<rto_local_move::LocalMoveAction> Server;
typedef enum { Idle, Moving, Rotating, Finished } State;

class RTOLocalMoveServer
{
public:
	RTOLocalMoveServer();
	~RTOLocalMoveServer();

private:
	ros::NodeHandle nh_;

	ros::Subscriber odometry_sub_;

	ros::Publisher cmd_vel_pub_;

	Server server_;

	State state_;

	rto_local_move::LocalMoveResult result_;
	rto_local_move::LocalMoveFeedback feedback_;

	geometry_msgs::Twist cmd_vel_msg_;
	nav_msgs::Odometry current_odom_msg_;
	nav_msgs::Odometry start_odom_msgs_;

	double curr_x_, curr_y_, curr_phi_, prev_phi_;
	double dist_moved_x_, dist_moved_y_, dist_rotated_;
	double forward_goal_x_, forward_goal_y_, rotation_goal_;
	double start_x_, start_y_, start_phi_;

	std::vector<geometry_msgs::Point32> forward_vel_vector_;
	std::vector<geometry_msgs::Point32> rotation_vel_vector_;

	bool odom_set_;

	void odomCallback( const nav_msgs::OdometryConstPtr& msg );
	void teleopActivatedCallback( const std_msgs::BoolConstPtr& msg );
	void execute( const rto_local_move::LocalMoveGoalConstPtr& goal );
	void setCmdVel( double vx, double vy, double omega );
	void controlLoop();
	bool acceptNewGoal( const rto_local_move::LocalMoveGoalConstPtr& goal );
	void readParameters( ros::NodeHandle& n);
	template< typename InputIterator > double linearApproximator(
			InputIterator iter, InputIterator end, const double x );

public:
	void spin();
};

#endif /* ROBOTINOLOCALMOVESERVER_H_ */
