// Generated by gencpp from file diagnostic_msgs/DiagnosticArray.msg
// DO NOT EDIT!


#ifndef DIAGNOSTIC_MSGS_MESSAGE_DIAGNOSTICARRAY_H
#define DIAGNOSTIC_MSGS_MESSAGE_DIAGNOSTICARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace diagnostic_msgs
{
template <class ContainerAllocator>
struct DiagnosticArray_
{
  typedef DiagnosticArray_<ContainerAllocator> Type;

  DiagnosticArray_()
    : header()
    , status()  {
    }
  DiagnosticArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::diagnostic_msgs::DiagnosticStatus_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::diagnostic_msgs::DiagnosticStatus_<ContainerAllocator> >::other >  _status_type;
  _status_type status;




  typedef boost::shared_ptr< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> const> ConstPtr;

}; // struct DiagnosticArray_

typedef ::diagnostic_msgs::DiagnosticArray_<std::allocator<void> > DiagnosticArray;

typedef boost::shared_ptr< ::diagnostic_msgs::DiagnosticArray > DiagnosticArrayPtr;
typedef boost::shared_ptr< ::diagnostic_msgs::DiagnosticArray const> DiagnosticArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace diagnostic_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'diagnostic_msgs': ['/home/ubuntu/Peach_Bot/catkin_ws/src/common_msgs/diagnostic_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "60810da900de1dd6ddd437c3503511da";
  }

  static const char* value(const ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x60810da900de1dd6ULL;
  static const uint64_t static_value2 = 0xddd437c3503511daULL;
};

template<class ContainerAllocator>
struct DataType< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "diagnostic_msgs/DiagnosticArray";
  }

  static const char* value(const ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message is used to send diagnostic information about the state of the robot\n\
Header header #for timestamp\n\
DiagnosticStatus[] status # an array of components being reported on\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: diagnostic_msgs/DiagnosticStatus\n\
# This message holds the status of an individual component of the robot.\n\
# \n\
\n\
# Possible levels of operations\n\
byte OK=0\n\
byte WARN=1\n\
byte ERROR=2\n\
byte STALE=3\n\
\n\
byte level # level of operation enumerated above \n\
string name # a description of the test/component reporting\n\
string message # a description of the status\n\
string hardware_id # a hardware unique string\n\
KeyValue[] values # an array of values associated with the status\n\
\n\
\n\
================================================================================\n\
MSG: diagnostic_msgs/KeyValue\n\
string key # what to label this value when viewing\n\
string value # a value to track over time\n\
";
  }

  static const char* value(const ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct DiagnosticArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::diagnostic_msgs::DiagnosticArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status[]" << std::endl;
    for (size_t i = 0; i < v.status.size(); ++i)
    {
      s << indent << "  status[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::diagnostic_msgs::DiagnosticStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.status[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DIAGNOSTIC_MSGS_MESSAGE_DIAGNOSTICARRAY_H
