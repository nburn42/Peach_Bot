// Generated by gencpp from file rosgraph_msgs/TopicStatistics.msg
// DO NOT EDIT!


#ifndef ROSGRAPH_MSGS_MESSAGE_TOPICSTATISTICS_H
#define ROSGRAPH_MSGS_MESSAGE_TOPICSTATISTICS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosgraph_msgs
{
template <class ContainerAllocator>
struct TopicStatistics_
{
  typedef TopicStatistics_<ContainerAllocator> Type;

  TopicStatistics_()
    : topic()
    , node_pub()
    , node_sub()
    , window_start()
    , window_stop()
    , delivered_msgs(0)
    , dropped_msgs(0)
    , traffic(0)
    , period_mean()
    , period_stddev()
    , period_max()
    , stamp_age_mean()
    , stamp_age_stddev()
    , stamp_age_max()  {
    }
  TopicStatistics_(const ContainerAllocator& _alloc)
    : topic(_alloc)
    , node_pub(_alloc)
    , node_sub(_alloc)
    , window_start()
    , window_stop()
    , delivered_msgs(0)
    , dropped_msgs(0)
    , traffic(0)
    , period_mean()
    , period_stddev()
    , period_max()
    , stamp_age_mean()
    , stamp_age_stddev()
    , stamp_age_max()  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _topic_type;
  _topic_type topic;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_pub_type;
  _node_pub_type node_pub;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _node_sub_type;
  _node_sub_type node_sub;

   typedef ros::Time _window_start_type;
  _window_start_type window_start;

   typedef ros::Time _window_stop_type;
  _window_stop_type window_stop;

   typedef int32_t _delivered_msgs_type;
  _delivered_msgs_type delivered_msgs;

   typedef int32_t _dropped_msgs_type;
  _dropped_msgs_type dropped_msgs;

   typedef int32_t _traffic_type;
  _traffic_type traffic;

   typedef ros::Duration _period_mean_type;
  _period_mean_type period_mean;

   typedef ros::Duration _period_stddev_type;
  _period_stddev_type period_stddev;

   typedef ros::Duration _period_max_type;
  _period_max_type period_max;

   typedef ros::Duration _stamp_age_mean_type;
  _stamp_age_mean_type stamp_age_mean;

   typedef ros::Duration _stamp_age_stddev_type;
  _stamp_age_stddev_type stamp_age_stddev;

   typedef ros::Duration _stamp_age_max_type;
  _stamp_age_max_type stamp_age_max;




  typedef boost::shared_ptr< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> const> ConstPtr;

}; // struct TopicStatistics_

typedef ::rosgraph_msgs::TopicStatistics_<std::allocator<void> > TopicStatistics;

typedef boost::shared_ptr< ::rosgraph_msgs::TopicStatistics > TopicStatisticsPtr;
typedef boost::shared_ptr< ::rosgraph_msgs::TopicStatistics const> TopicStatisticsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosgraph_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rosgraph_msgs': ['/home/ubuntu/Peach_Bot/catkin_ws/src/ros_comm_msgs/rosgraph_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "10152ed868c5097a5e2e4a89d7daa710";
  }

  static const char* value(const ::rosgraph_msgs::TopicStatistics_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x10152ed868c5097aULL;
  static const uint64_t static_value2 = 0x5e2e4a89d7daa710ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosgraph_msgs/TopicStatistics";
  }

  static const char* value(const ::rosgraph_msgs::TopicStatistics_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# name of the topic\n\
string topic\n\
\n\
# node id of the publisher\n\
string node_pub\n\
\n\
# node id of the subscriber\n\
string node_sub\n\
\n\
# the statistics apply to this time window\n\
time window_start\n\
time window_stop\n\
\n\
# number of messages delivered during the window\n\
int32 delivered_msgs\n\
# numbers of messages dropped during the window\n\
int32 dropped_msgs\n\
\n\
# traffic during the window, in bytes\n\
int32 traffic\n\
\n\
# mean/stddev/max period between two messages\n\
duration period_mean\n\
duration period_stddev\n\
duration period_max\n\
\n\
# mean/stddev/max age of the message based on the\n\
# timestamp in the message header. In case the\n\
# message does not have a header, it will be 0.\n\
duration stamp_age_mean\n\
duration stamp_age_stddev\n\
duration stamp_age_max\n\
";
  }

  static const char* value(const ::rosgraph_msgs::TopicStatistics_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.topic);
      stream.next(m.node_pub);
      stream.next(m.node_sub);
      stream.next(m.window_start);
      stream.next(m.window_stop);
      stream.next(m.delivered_msgs);
      stream.next(m.dropped_msgs);
      stream.next(m.traffic);
      stream.next(m.period_mean);
      stream.next(m.period_stddev);
      stream.next(m.period_max);
      stream.next(m.stamp_age_mean);
      stream.next(m.stamp_age_stddev);
      stream.next(m.stamp_age_max);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TopicStatistics_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosgraph_msgs::TopicStatistics_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosgraph_msgs::TopicStatistics_<ContainerAllocator>& v)
  {
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.topic);
    s << indent << "node_pub: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.node_pub);
    s << indent << "node_sub: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.node_sub);
    s << indent << "window_start: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.window_start);
    s << indent << "window_stop: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.window_stop);
    s << indent << "delivered_msgs: ";
    Printer<int32_t>::stream(s, indent + "  ", v.delivered_msgs);
    s << indent << "dropped_msgs: ";
    Printer<int32_t>::stream(s, indent + "  ", v.dropped_msgs);
    s << indent << "traffic: ";
    Printer<int32_t>::stream(s, indent + "  ", v.traffic);
    s << indent << "period_mean: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.period_mean);
    s << indent << "period_stddev: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.period_stddev);
    s << indent << "period_max: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.period_max);
    s << indent << "stamp_age_mean: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.stamp_age_mean);
    s << indent << "stamp_age_stddev: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.stamp_age_stddev);
    s << indent << "stamp_age_max: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.stamp_age_max);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSGRAPH_MSGS_MESSAGE_TOPICSTATISTICS_H
