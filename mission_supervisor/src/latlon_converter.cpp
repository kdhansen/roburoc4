// Latitude longitude converter for the Mission Supervisor for the RobuROC4.
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
#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <GeographicLib/UTMUPS.hpp> // Conversion between lat/long and UTM

int main(int argc, char** argv)
{
  ros::init(argc, argv, "latlon_converter");

  if (3 != argc)
  {
    std::cout << "Supply both lat and lon." << argc << std::endl;
    return 1;
  }

  // Read lat/lon from arguments
  std::stringstream convert_lat(argv[1]);
  double latitude;
  convert_lat >> latitude;

  std::stringstream convert_lon(argv[2]);
  double longitude;
  convert_lon >> longitude;
  
  // Convert to UTM
  int zone;
  bool northp;
  double utmx,utmy;
  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utmx, utmy);

  // Publish the point
  ros::NodeHandle _ph("~");
  bool doLatching = true;
  ros::Publisher _pointPub = _ph.advertise<geometry_msgs::Point>("converted_point", 1, doLatching);
  geometry_msgs::Point converted_point;
  converted_point.x = utmx;
  converted_point.y = utmy;
  converted_point.z = 0;
  _pointPub.publish(converted_point);

  ros::Duration(3).sleep();

  return 0;
}
