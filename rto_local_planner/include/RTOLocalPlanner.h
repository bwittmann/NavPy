/*
 * RTOLocalPlanner.h
 *
 *  Created on: Feb 20, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOLOCALPLANNER_H_
#define ROBOTINOLOCALPLANNER_H_

#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace rto_local_planner
{
	class RTOLocalPlanner: public nav_core::BaseLocalPlanner
	{
	public:
		RTOLocalPlanner();
		~RTOLocalPlanner();

		void initialize( std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros );
		bool computeVelocityCommands( geometry_msgs::Twist& cmd_vel);
		bool isGoalReached();
		bool setPlan( const std::vector<geometry_msgs::PoseStamped>& global_plan );

	private:
		void odomCallback(const nav_msgs::OdometryConstPtr& msg);
		void publishNextHeading(bool show = true);
		bool rotateToStart( geometry_msgs::Twist& cmd_vel );
		bool move( geometry_msgs::Twist& cmd_vel );
		bool rotateToGoal( geometry_msgs::Twist& cmd_vel );
		void computeNextHeadingIndex();
		double calLinearVel();
		double calRotationVel( double rotation );
		double linearDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 );
		double mapToMinusPIToPI( double angle );

		typedef enum { RotatingToStart, Moving, RotatingToGoal, Finished } State;

		tf2_ros::Buffer* tf_;

		std::vector<geometry_msgs::PoseStamped> global_plan_;

		geometry_msgs::PoseStamped base_odom_;

		ros::Subscriber odom_sub_;

		ros::Publisher next_heading_pub_;

		State state_;

		boost::mutex odom_lock_;

		int curr_heading_index_, next_heading_index_;

		// Parameters
		double heading_lookahead_;
		double max_linear_vel_, min_linear_vel_;
		double max_rotation_vel_, min_rotation_vel_;
		double yaw_goal_tolerance_, xy_goal_tolerance_;
		int num_window_points_;

	};
}

#endif /* ROBOTINOLOCALPLANNER_H_ */
