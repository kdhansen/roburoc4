/* Auto-generated by genmsg_cpp for file /home/roburoc4operator/aseta_demo_workspace/roburoc4/roburoc4_controllers/msg/LineSegment.msg */
#ifndef ROBUROC4_CONTROLLERS_MESSAGE_LINESEGMENT_H
#define ROBUROC4_CONTROLLERS_MESSAGE_LINESEGMENT_H
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

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point.h"

namespace roburoc4_controllers
{
template <class ContainerAllocator>
struct LineSegment_ {
  typedef LineSegment_<ContainerAllocator> Type;

  LineSegment_()
  : start()
  , end()
  {
  }

  LineSegment_(const ContainerAllocator& _alloc)
  : start(_alloc)
  , end(_alloc)
  {
  }

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _start_type;
   ::geometry_msgs::Point_<ContainerAllocator>  start;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _end_type;
   ::geometry_msgs::Point_<ContainerAllocator>  end;


  typedef boost::shared_ptr< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roburoc4_controllers::LineSegment_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct LineSegment
typedef  ::roburoc4_controllers::LineSegment_<std::allocator<void> > LineSegment;

typedef boost::shared_ptr< ::roburoc4_controllers::LineSegment> LineSegmentPtr;
typedef boost::shared_ptr< ::roburoc4_controllers::LineSegment const> LineSegmentConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::roburoc4_controllers::LineSegment_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::roburoc4_controllers::LineSegment_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace roburoc4_controllers

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::roburoc4_controllers::LineSegment_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ad6f4eea34a193d38008f1d4053cce66";
  }

  static const char* value(const  ::roburoc4_controllers::LineSegment_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xad6f4eea34a193d3ULL;
  static const uint64_t static_value2 = 0x8008f1d4053cce66ULL;
};

template<class ContainerAllocator>
struct DataType< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > {
  static const char* value() 
  {
    return "roburoc4_controllers/LineSegment";
  }

  static const char* value(const  ::roburoc4_controllers::LineSegment_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Point start\n\
geometry_msgs/Point end\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
";
  }

  static const char* value(const  ::roburoc4_controllers::LineSegment_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::roburoc4_controllers::LineSegment_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::roburoc4_controllers::LineSegment_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.start);
    stream.next(m.end);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct LineSegment_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roburoc4_controllers::LineSegment_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::roburoc4_controllers::LineSegment_<ContainerAllocator> & v) 
  {
    s << indent << "start: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.start);
    s << indent << "end: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.end);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBUROC4_CONTROLLERS_MESSAGE_LINESEGMENT_H

