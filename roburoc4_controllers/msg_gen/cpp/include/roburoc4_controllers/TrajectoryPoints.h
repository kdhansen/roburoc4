/* Auto-generated by genmsg_cpp for file /home/roburoc4operator/aseta_demo_workspace/roburoc4/roburoc4_controllers/msg/TrajectoryPoints.msg */
#ifndef ROBUROC4_CONTROLLERS_MESSAGE_TRAJECTORYPOINTS_H
#define ROBUROC4_CONTROLLERS_MESSAGE_TRAJECTORYPOINTS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

namespace roburoc4_controllers
{
template <class ContainerAllocator>
struct TrajectoryPoints_ {
  typedef TrajectoryPoints_<ContainerAllocator> Type;

  TrajectoryPoints_()
  : pose()
  , velocity()
  {
  }

  TrajectoryPoints_(const ContainerAllocator& _alloc)
  : pose(_alloc)
  , velocity(_alloc)
  {
  }

  typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
   ::geometry_msgs::Pose2D_<ContainerAllocator>  pose;

  typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _velocity_type;
   ::geometry_msgs::Twist_<ContainerAllocator>  velocity;


  typedef boost::shared_ptr< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct TrajectoryPoints
typedef  ::roburoc4_controllers::TrajectoryPoints_<std::allocator<void> > TrajectoryPoints;

typedef boost::shared_ptr< ::roburoc4_controllers::TrajectoryPoints> TrajectoryPointsPtr;
typedef boost::shared_ptr< ::roburoc4_controllers::TrajectoryPoints const> TrajectoryPointsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace roburoc4_controllers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2558566a87d8b4a2995056624a439763";
  }

  static const char* value(const  ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2558566a87d8b4a2ULL;
  static const uint64_t static_value2 = 0x995056624a439763ULL;
};

template<class ContainerAllocator>
struct DataType< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roburoc4_controllers/TrajectoryPoints";
  }

  static const char* value(const  ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# States of the Robot\n\
geometry_msgs/Pose2D pose\n\
geometry_msgs/Twist velocity\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pose);
    stream.next(m.velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct TrajectoryPoints_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::roburoc4_controllers::TrajectoryPoints_<ContainerAllocator> & v) 
  {
    s << indent << "pose: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBUROC4_CONTROLLERS_MESSAGE_TRAJECTORYPOINTS_H
