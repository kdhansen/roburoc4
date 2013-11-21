// Joystick teleoperations for the RobuROC4.
//
// A joystick teleoperations node for differential drive vehicles. Made with inspiration from 
// Melonee Wise's turtlebot_apps - joystick teleop node.
// This is a part of the ASETA project, Aalborg University.
// 
// Copyright 2012 Karl Damkjær Hansen
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
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

class TeleopDiff
{
  public:
    TeleopDiff();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void publish(void);

    ros::NodeHandle nh_;

    int angularAxis_, linearAxis_, deadManButton_;
    double angularScale_, linearScale_;

    ros::Publisher velPub_;
    ros::Subscriber joySub_;

    geometry_msgs::Twist currVel_;
    boost::mutex pubMutex_;
    bool deadManPressed_;
    ros::Timer timer_;
};


TeleopDiff::TeleopDiff():
  angularAxis_(0),
  linearAxis_(1),
  deadManButton_(0),
  angularScale_(1),
  linearScale_(0.5),
  deadManPressed_(false)
{
  nh_.param("angularAxis", angularAxis_, angularAxis_);
  nh_.param("linearAxis", linearAxis_, linearAxis_);
  nh_.param("angularScale", angularScale_, angularScale_);
  nh_.param("linearScale", linearScale_, linearScale_);
  nh_.param("deadManButton", deadManButton_, deadManButton_);

  joySub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopDiff::joyCallback, this);

  velPub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TeleopDiff::publish, this));
}


void TeleopDiff::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = joy->axes[angularAxis_] * angularScale_;
  vel.linear.x = joy->axes[linearAxis_] * linearScale_;
  currVel_ = vel;
  deadManPressed_ = joy->buttons[deadManButton_];
}


void TeleopDiff::publish(void)
{
  boost::mutex::scoped_lock lock(pubMutex_);

  if (deadManPressed_)
  {
    velPub_.publish(currVel_);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kdh_teleop");
  TeleopDiff teleopDiff;

  ros::spin();
}
