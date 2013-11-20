// UTM file reader for the Mission Supervisor for the RobuROC4.
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

#include <sstream>
#include <fstream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "utm_file_reader");

  if (2 != argc)
  {
    std::cout << "Usage: " << argv[0] << " <file with UTM coordinates> [~waypoints:=remap]" << std::endl;
    return 1;
  }

  // Open file
  std::ifstream utm_file(argv[1]);

  // Publish the points
  ros::NodeHandle _ph("~");
  bool doLatching = true;
  ros::Publisher _pointPub = _ph.advertise<geometry_msgs::Point>("waypoints", 100, doLatching);
  ros::Duration(1.0).sleep();
  while(utm_file.good())
  {
    double x, y;
    utm_file >> x >> y;
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    _pointPub.publish(point);
    ros::Duration(0.01).sleep();
  }

  // Close up
  utm_file.close();

  // Latch a couple of seconds
  ros::Duration(2).sleep();

  return 0;
}
