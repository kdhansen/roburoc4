// Open loop playback device for the RobuROC4.
//
// This is a tool that plays back a sequence of control signals.
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

// ROS stuff
#include <ros/ros.h>
#include <ros/console.h>

// Messages
#include <geometry_msgs/Twist.h>

// File operation
#include <fstream>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "open_loop_playback");

	if (2 != argc)
	{
		std::cout << "Usage: open_loop_playback <file>" << std::endl;
		return 1;
	}

	ros::NodeHandle nh;
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Open file
	std::ifstream my_file(argv[1]);

	// Read values
	ros::Rate r(10);
	while(my_file.good() && ros::ok())
	{
		double lin_speed, ang_speed;
		my_file >> lin_speed >> ang_speed;
		geometry_msgs::Twist t;
		t.linear.x = lin_speed;
		t.angular.z = ang_speed;
		vel_pub.publish(t);
		r.sleep();
		ros::spinOnce();
	}

	// Close file
	my_file.close();

	return 0;
}
