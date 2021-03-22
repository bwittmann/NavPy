/*
 * JoystickTeleop.cpp
 *
 *  Created on: 16.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "JoystickTeleop.h"

JoystickTeleop::JoystickTeleop():
	nh_("~")
{
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	joy_sub_ = nh_.subscribe("/joy", 1, &JoystickTeleop::joyCallback, this);

	readParams( nh_ );
}

JoystickTeleop::~JoystickTeleop()
{
	cmd_vel_pub_.shutdown();
	joy_sub_.shutdown();
}

void JoystickTeleop::readParams( ros::NodeHandle& n)
{
	n.param<int>("axis_linear_x", axis_linear_x_, 1);
	n.param<int>("axis_linear_y", axis_linear_y_, 0);
	n.param<int>("axis_angular", axis_angular_, 3);
	n.param<double>("scale_linear", scale_linear_, 0.3);
	n.param<double>("scale_angular", scale_angular_, 0.6);
	n.param<double>("scale_turbo", scale_turbo_, 2.0);
	n.param<double>("deadzone", deadzone_, 0.1);
}

void JoystickTeleop::joyCallback( const sensor_msgs::JoyConstPtr& msg)
{
	if( msg->axes.size() < 3)
	{
		ROS_ERROR( "Too few joystick axes: %lu (expected more than 3)", msg->axes.size() );
		return;
	}

	auto axes = msg->axes;

	// Set turbo to 2 if button 'LB' is pressed, 1 otherwise
	float turbo = 1.0;
	if (msg->buttons[4] == 1)
		turbo = scale_turbo_;

	for (int i=0; i<axes.size(); i++)
	{
		if ( (axes[i] > 0 && axes[i] < deadzone_) ||
			 (axes[i] < 0 && axes[i] > -deadzone_) )
		{
			axes[i] = 0.0;
		}
	}

	// Use input only if button 'RB' is pressed otherwise set everything to zero 
	if (msg->buttons[5] == 1)
	{
		cmd_vel_msg_.linear.x = axes[axis_linear_x_] * scale_linear_ * turbo;
		cmd_vel_msg_.linear.y = axes[axis_linear_y_] * scale_linear_ * turbo;
		cmd_vel_msg_.angular.z = axes[axis_angular_] * scale_angular_ * turbo;
	}
	else
	{
		cmd_vel_msg_.linear.x = 0;
		cmd_vel_msg_.linear.y = 0;
		cmd_vel_msg_.angular.z = 0;
	}
}

void JoystickTeleop::spin()
{
	ros::Rate loop_rate(5);
	while( nh_.ok() )
	{
		cmd_vel_pub_.publish( cmd_vel_msg_ );
		ros::spinOnce();
		loop_rate.sleep();
	}
}
