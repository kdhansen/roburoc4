// Circle publisher for the RobuROC4.
// This is a tool to test the trajectory follower.
//
// This is a part of the ASETA project, Aalborg University.
// 
// Copyright 2013 Karl Damkjær Hansen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as publishSegmented by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <roburoc4/States.h>
#include <roburoc4/Trajectory.h>
#include <roburoc4/TrajectoryPoints.h>

roburoc4::States current_state;
bool got_state = false;

void getState(const roburoc4::StatesConstPtr& state)
{
	if(!got_state)
	{
		current_state = *state;
		got_state = true;
		ROS_INFO("Got the state. [x:%f, y:%f, theta:%f]", state->x, state->y, state->theta);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circle_tool");
	if (3 != argc)
	{
		std::cout << "Usage: circle_tool <radius> <speed> [~state_estimate:=remap] [~trajectory:=remap]" << std::endl;
		return 1;
	}

	ros::NodeHandle ph("~");
	ros::Publisher trajectory_pub = ph.advertise<roburoc4::Trajectory>("trajectory", 1);
	ros::Subscriber state_sub = ph.subscribe("state_estimate", 1, getState);

	double radius = atof(argv[1]);
	double speed = atof(argv[2]);

	ROS_INFO("Waiting for state estimate...");
	while(!got_state && ros::ok())
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}

	ROS_INFO("Computing circle.");
	// Number of samples on circle.
	double control_loop_rate = 20;
	double angular_speed = speed / radius;
	double radians_per_sample = angular_speed / control_loop_rate;
	double num_samples_on_circle = 2 * 3.14159 / radians_per_sample;
	
	// Center of circle (for a left turn).
	double center_x = -radius * sin(current_state.theta) + current_state.x;
	double center_y = radius * cos(current_state.theta) + current_state.y;
	ROS_INFO("Center at [%f %f]", center_x, center_y);

	// Fill trajectory.
	roburoc4::Trajectory circle_trajectory;
	double angle = current_state.theta;
	for (int i = 0; i < num_samples_on_circle; ++i)
	{
		roburoc4::TrajectoryPoints tp;

		angle = fmod(angle+radians_per_sample, 2*3.14159);
		tp.pose.theta = angle;
		tp.pose.x = center_x + radius * sin(angle);
		tp.pose.y = center_y - radius * cos(angle);

		tp.velocity.linear.x = speed;
		tp.velocity.angular.z = angular_speed;

		circle_trajectory.points.push_back(tp);
	}

	ROS_INFO("Publishing...");
	trajectory_pub.publish(circle_trajectory);

	return 0;
}
