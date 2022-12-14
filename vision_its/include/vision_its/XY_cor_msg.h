// Generated by gencpp from file vision/XY_cor_msg.msg
// DO NOT EDIT!


#ifndef VISION_MESSAGE_XY_COR_MSG_H
#define VISION_MESSAGE_XY_COR_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vision
{
template <class ContainerAllocator>
struct XY_cor_msg_
{
  typedef XY_cor_msg_<ContainerAllocator> Type;

  XY_cor_msg_()
    : x(0.0)
    , y(0.0)  {
    }
  XY_cor_msg_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::vision::XY_cor_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision::XY_cor_msg_<ContainerAllocator> const> ConstPtr;

}; // struct XY_cor_msg_

typedef ::vision::XY_cor_msg_<std::allocator<void> > XY_cor_msg;

typedef boost::shared_ptr< ::vision::XY_cor_msg > XY_cor_msgPtr;
typedef boost::shared_ptr< ::vision::XY_cor_msg const> XY_cor_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision::XY_cor_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision::XY_cor_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vision::XY_cor_msg_<ContainerAllocator1> & lhs, const ::vision::XY_cor_msg_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vision::XY_cor_msg_<ContainerAllocator1> & lhs, const ::vision::XY_cor_msg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vision::XY_cor_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision::XY_cor_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::XY_cor_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::XY_cor_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::XY_cor_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::XY_cor_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision::XY_cor_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::vision::XY_cor_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision::XY_cor_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision/XY_cor_msg";
  }

  static const char* value(const ::vision::XY_cor_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision::XY_cor_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::vision::XY_cor_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision::XY_cor_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct XY_cor_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision::XY_cor_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision::XY_cor_msg_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MESSAGE_XY_COR_MSG_H
