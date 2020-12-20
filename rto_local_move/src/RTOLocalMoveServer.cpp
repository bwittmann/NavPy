/*
 * RTOLocalMoveServer.cpp
 *
 *  Created on: 14.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "RTOLocalMoveServer.h"

#include <tf/transform_datatypes.h>

RTOLocalMoveServer::RTOLocalMoveServer():
	nh_("~"),
	server_ ( nh_, "/local_move",
		boost::bind( &RTOLocalMoveServer::execute, this, _1 ), false ),
	curr_x_( 0.0 ),
	curr_y_( 0.0 ),
	curr_phi_( 0.0 ),
	prev_phi_( 0.0 ),
	dist_moved_x_( 0.0 ),
	dist_moved_y_( 0.0 ),
	dist_rotated_( 0.0 ),
	forward_goal_x_( 0.0 ),
	forward_goal_y_( 0.0 ),
	rotation_goal_( 0.0 ),
	start_x_( 0.0 ),
	start_y_( 0.0 ),
	start_phi_( 0.0 ),
	odom_set_(false )
{
	odometry_sub_ = nh_.subscribe( "/odom", 1,
			&RTOLocalMoveServer::odomCallback, this );

	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1 );

	state_ = Idle;

	readParameters( nh_ );
}

RTOLocalMoveServer::~RTOLocalMoveServer()
{
	odometry_sub_.shutdown();
	cmd_vel_pub_.shutdown();
	server_.shutdown();
}

void RTOLocalMoveServer::odomCallback( const nav_msgs::OdometryConstPtr& msg )
{
	curr_x_ = msg->pose.pose.position.x;
	curr_y_ = msg->pose.pose.position.y;
	curr_phi_ = tf::getYaw( msg->pose.pose.orientation );

	if( odom_set_ == false )
	{
		ROS_INFO( "Odometry initialized" );
		odom_set_ = true;
		prev_phi_ = curr_phi_;

		server_.start();
	}

	while( curr_phi_ - prev_phi_ < PI )
	{
		curr_phi_ += 2 * PI;
	}

	while( curr_phi_ - prev_phi_ > PI )
	{
		curr_phi_ -= 2 * PI;
	}

	dist_rotated_ += curr_phi_ - prev_phi_;
	prev_phi_ = curr_phi_;
	dist_moved_x_ = curr_x_ - start_x_;
	dist_moved_y_ = curr_y_ - start_y_;

	double distance_moved_x_temp = dist_moved_x_;

	dist_moved_x_ = dist_moved_x_ * cos( -start_phi_ ) - dist_moved_y_ * sin( -start_phi_ );
	dist_moved_y_ = dist_moved_y_ * cos( -start_phi_ ) + distance_moved_x_temp * sin( -start_phi_ );
}

void RTOLocalMoveServer::execute( const rto_local_move::LocalMoveGoalConstPtr& goal )
{
	ros::Rate loop_rate( 10 );

	if( !acceptNewGoal( goal ) )
	{
		ROS_WARN( "Goal not accepted" );
		return;
	}

	while( nh_.ok() )
	{
		if( server_.isPreemptRequested() )
		{
			if( server_.isNewGoalAvailable() )
			{
				if( acceptNewGoal( server_.acceptNewGoal() ) == false )
				{
					return;
				}
			}
			else
			{
				ROS_INFO( "Cancel request" );
				server_.setPreempted();
				state_ = Idle;
				setCmdVel( 0, 0, 0 );

				return;
			}
		}

		controlLoop();

		if( state_ == Finished )
		{
			setCmdVel( 0, 0, 0 );
			cmd_vel_pub_.publish( cmd_vel_msg_ );

			result_.goal_reached = true;
			server_.setSucceeded( result_ );
			state_ = Idle;
			ROS_INFO( "Local move execution complete: (x[m], y[m], phi[deg]): (%f, %f, %f)",
					dist_moved_x_, dist_moved_y_, ( dist_rotated_ * 180 ) / PI );

			return;
		}

		if( state_ != Idle )
		{
			cmd_vel_pub_.publish( cmd_vel_msg_ );
			server_.publishFeedback( feedback_ );
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	setCmdVel( 0, 0, 0 );
	cmd_vel_pub_.publish( cmd_vel_msg_ );

	server_.setAborted(rto_local_move::LocalMoveResult(),
			"Aborting on the goal because the node has been killed");
}

void RTOLocalMoveServer::setCmdVel( double vx, double vy, double omega )
{
	cmd_vel_msg_.linear.x = vx;
	cmd_vel_msg_.linear.y = vy;
	cmd_vel_msg_.angular.z = omega;
}

void RTOLocalMoveServer::spin()
{
	ros::Rate loop_rate ( 5 );

	ROS_INFO( "RTO Local Move Server up and running" );
	while( nh_.ok() )
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void RTOLocalMoveServer::controlLoop()
{
	setCmdVel( 0, 0, 0);
	switch( state_ )
	{
	case Moving:
	{
		double dist_driven_x = fabs( dist_moved_x_ );
		double dist_driven_y = fabs( dist_moved_y_ );

		ROS_DEBUG( "Moved (x, y) = (%f, %f)", dist_driven_x, dist_driven_y );

		double forward_goal_x_abs = fabs( forward_goal_x_ );
		double forward_goal_y_abs = fabs( forward_goal_y_ );

		double vel_x = 0.0;
		double vel_y = 0.0;

		if( dist_driven_x < forward_goal_x_abs )
		{
			double distance_to_go = forward_goal_x_abs - dist_driven_x;
			feedback_.forward_dist_x = distance_to_go;

			double vel = linearApproximator(
					forward_vel_vector_.begin(),
					forward_vel_vector_.end(),
					distance_to_go );

			if( forward_goal_x_ < 0.0 )
			{
				vel = -vel;
			}
			vel_x = vel;
		}

		if( dist_driven_y < forward_goal_y_abs )
		{
			double distance_to_go = forward_goal_y_abs - dist_driven_y;
			feedback_.forward_dist_y = distance_to_go;

			double vel = linearApproximator(
					forward_vel_vector_.begin(),
					forward_vel_vector_.end(),
					distance_to_go );

			if( forward_goal_y_ < 0.0 )
			{
				vel = -vel;
			}
			vel_y = vel;
		}

		if( 0.0 == vel_x && 0.0 == vel_y )
		{
			state_ = Rotating;
		}
		else
		{
			setCmdVel( vel_x, vel_y, 0 );
		}
	}
	break;

	case Rotating:
	{
		double dist_rotated_abs = fabs( dist_rotated_ );
		double rotation_goal_abs = fabs( rotation_goal_ );

		if( dist_rotated_abs < rotation_goal_abs )
		{
			ROS_DEBUG( "Rotated %f degrees", ( dist_rotated_ * 180 ) / PI );

			double distance_to_go = rotation_goal_abs - dist_rotated_abs;
			feedback_.rotation_dist = distance_to_go;

			double vel = linearApproximator(
					rotation_vel_vector_.begin(),
					rotation_vel_vector_.end(),
					distance_to_go );

			if( rotation_goal_ < 0.0 )
			{
				vel = -vel;
			}
			setCmdVel( 0, 0, vel );
		}
		else
		{
			state_ = Finished;
		}
	}
	break;

	default:
		setCmdVel( 0, 0, 0 );
		break;
	}
}

bool RTOLocalMoveServer::acceptNewGoal( const rto_local_move::LocalMoveGoalConstPtr& goal )
{
	if( odom_set_ )
	{
		forward_goal_x_ = goal->move_x;
		forward_goal_y_ = goal->move_y;
		rotation_goal_ = goal->rotation;

		ROS_INFO( "Local move execution start: (x[m], y[m], phi[deg]): (%f, %f, %f)",
				forward_goal_x_, forward_goal_y_, (rotation_goal_ * 180) / PI );

		start_x_ = curr_x_;
		start_y_ = curr_y_;
		start_phi_ = curr_phi_;
		dist_moved_x_ = 0.0;
		dist_moved_y_ = 0.0;
		dist_rotated_ = 0.0;

		if( fabs( forward_goal_x_ ) > 0.02 || fabs( forward_goal_y_ ) > 0.02 )
		{
			state_ = Moving;
		}
		else if( fabs( rotation_goal_ ) > 0.01 )
		{
			state_ = Rotating;
		}
		else
		{
			state_ = Finished;
		}

		return true;
	}
	else
	{
		ROS_ERROR( "Odometry not initialized" );
		return false;
	}
}

void RTOLocalMoveServer::readParameters( ros::NodeHandle& n)
{
	double min_forward_vel;
	n.param( "min_forward_vel", min_forward_vel, 0.05 );

	double max_forward_vel;
	n.param( "max_forward_vel", max_forward_vel, 0.1 );

	double min_forward_vel_distance;
	n.param( "min_forward_vel_distance", min_forward_vel_distance, 0.1 );

	double max_forward_vel_distance;
	n.param( "max_forward_vel_distance", max_forward_vel_distance, 0.5 );

	geometry_msgs::Point32 point;
	point.x = 0.0;
	point.y = min_forward_vel;
	forward_vel_vector_.push_back( point );

	point.x = min_forward_vel_distance;
	point.y = min_forward_vel;
	forward_vel_vector_.push_back( point );

	point.x = max_forward_vel_distance;
	point.y = max_forward_vel;
	forward_vel_vector_.push_back( point );

	double min_rotational_vel;
	n.param( "min_rotational_vel", min_rotational_vel, 0.04 );

	double max_rotational_vel;
	n.param( "max_rotational_vel", max_rotational_vel, 0.2 );

	double min_rotational_vel_distance;
	n.param( "min_rotational_vel_distance", min_rotational_vel_distance, 0.05 );

	double max_rotational_vel_distance;
	n.param( "max_rotational_vel_distance", max_rotational_vel_distance, 0.2 );

	point.x = 0.0;
	point.y = min_rotational_vel;
	rotation_vel_vector_.push_back( point );

	point.x = min_rotational_vel_distance;
	point.y = min_rotational_vel;
	rotation_vel_vector_.push_back( point );

	point.x = max_rotational_vel_distance;
	point.y = max_rotational_vel;
	rotation_vel_vector_.push_back( point );
}

template< typename InputIterator > double RTOLocalMoveServer::linearApproximator(
		InputIterator iter, InputIterator end, const double x )
{
	double y = 0.0;

	if( iter != end )
	{
		if( x < (*iter).x )
		{
			y = (*iter).y;
		}
		else
		{
			while( end != iter )
			{
				const geometry_msgs::Point32& p = (*iter);
				++iter;

				if( x >= p.x )
				{
					if( end != iter )
					{
						if( x < (*iter).x )
						{
							const geometry_msgs::Point32& p2 = (*iter);
							double dx = p2.x - p.x;
							double dy = p2.y - p.y;

							if( 0.0 == dx )
							{
								y = p.y;
							}
							else
							{
								double a = dy / dx;
								y = a * ( x - p.x ) + p.y;
							}
						}
					}
					else
					{
						y = p.y;
					}
				}
			}
		}
	}

	return y;
}
