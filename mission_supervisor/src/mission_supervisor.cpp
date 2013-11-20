// Mission Supervisor for the RobuROC4.
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

#include <cmath>
#include <deque>
#include <ros/console.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Point.h>
#include <roburoc4_controllers/LineSegment.h>
#include <roburoc4_state_estimator/States.h>

class MissionSupervisor
{
  public:
    MissionSupervisor();
    ~MissionSupervisor(){};

  private:
    void stateEstimateCallback(const roburoc4_state_estimator::States::ConstPtr&);
    void newWaypointCallback(const geometry_msgs::Point::ConstPtr&);
    void publishSegment(void);

    ros::NodeHandle _nh;
    ros::NodeHandle _ph;

    ros::Subscriber _wpSub;
    ros::Subscriber _stateSub;
    ros::Publisher _segmentPub;
    ros::Timer _timer;

    std::deque<geometry_msgs::Point> _point_que;
    roburoc4_controllers::LineSegment _currentGoal;
    double _goal_threshold;
    double _new_waypoint_delay;

    unsigned int _wps_ever_added;

    boost::mutex _pubMutex;
};


MissionSupervisor::MissionSupervisor():
  _ph("~"),
  _goal_threshold(1),
  _new_waypoint_delay(0),
  _wps_ever_added(0)
{
  _ph.param("goal_threshold", _goal_threshold, _goal_threshold);
  _ph.param("new_waypoint_delay", _new_waypoint_delay, _new_waypoint_delay);

  _stateSub = _nh.subscribe<roburoc4_state_estimator::States>("state_estimate", 10, &MissionSupervisor::stateEstimateCallback, this);
  _wpSub = _ph.subscribe<geometry_msgs::Point>("new_waypoint", 10, &MissionSupervisor::newWaypointCallback, this);

  _segmentPub = _ph.advertise<roburoc4_controllers::LineSegment>("cmd_segment", 1);
  
  _timer = _nh.createTimer(ros::Duration(1), boost::bind(&MissionSupervisor::publishSegment, this));
}

void MissionSupervisor::stateEstimateCallback(const roburoc4_state_estimator::States::ConstPtr& state)
{
  double dx = _currentGoal.end.x - state->x;
  double dy = _currentGoal.end.y - state->y;
  double dist = sqrt(dx*dx + dy*dy);
  if (_goal_threshold > dist)
  {
    if (0 < _point_que.size())
    {
      _currentGoal.start = _currentGoal.end;
      _currentGoal.end = _point_que[0];
      _point_que.pop_front();
      ros::Duration(_new_waypoint_delay).sleep();
    }
  }
}

void MissionSupervisor::newWaypointCallback(const geometry_msgs::Point::ConstPtr& wp)
{
  if (0 == _wps_ever_added)
  {
    _currentGoal.start = *wp;
    ROS_INFO("Starting waypoint set. [%f %f]", wp->x, wp->y);
  }
  else if (1 == _wps_ever_added)
  {
    _currentGoal.end = *wp;
    ROS_INFO("Goal waypoint set. [%f %f]", wp->x, wp->y);
  }
  else
  {
    _point_que.push_back(geometry_msgs::Point(*wp));
    ROS_INFO("New Waypoint added to que. [%f %f]", wp->x, wp->y);
  }
  _wps_ever_added += 1;
}

void MissionSupervisor::publishSegment(void)
{
  if (2 <= _wps_ever_added)
  {
    boost::mutex::scoped_lock lock(_pubMutex);
    _segmentPub.publish(_currentGoal);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mission_supervisor");
  MissionSupervisor supervisor;

  ros::spin();
}
