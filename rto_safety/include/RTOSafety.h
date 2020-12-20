/*
 * RTOSafety.h
 *
 *  Created on: Mar 21, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOSAFETY_H_
#define ROBOTINOSAFETY_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>

class RTOSafety
{
public:
	RTOSafety();
	~RTOSafety();

	void spin();

private:
	ros::NodeHandle nh_;

	ros::Publisher cmd_vel_pub_;
	ros::Publisher e1_viz_pub_;
	ros::Publisher e2_viz_pub_;

	ros::Subscriber rto_cmd_vel_sub_;
	ros::Subscriber bumper_sub_;
	ros::Subscriber scan_sub_;

	geometry_msgs::Twist cmd_vel_msg_;

	visualization_msgs::Marker e1_viz_msg_, e2_viz_msg_;

	laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_;

	bool stop_bumper_, stop_laser_, slow_laser_;

	double scale_, dist_;

	// params
	double e1_major_radius_, e1_minor_radius_;
	double e2_major_radius_, e2_minor_radius_;
	int node_loop_rate_;

	void calcScale();
	void buildEllipseVizMsgs();

	void rtoCmdVelCallback(const geometry_msgs::TwistConstPtr& msg);
	void bumperCallback(const std_msgs::BoolConstPtr& msg);
	void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);

	void check(sensor_msgs::PointCloud cloud);

	void inE2(geometry_msgs::Point32 point);
	double solveE1(geometry_msgs::Point32 point);

	void visualizeEllipses(bool show = true );
};

#endif /* ROBOTINOSAFETY_H_ */
