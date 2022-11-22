// Generated by gencpp from file vision/RotatedRect.msg
// DO NOT EDIT!


#ifndef VISION_MESSAGE_ROTATEDRECT_H
#define VISION_MESSAGE_ROTATEDRECT_H


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
struct RotatedRect_
{
  typedef RotatedRect_<ContainerAllocator> Type;

  RotatedRect_()
    : is_exist(false)
    , x(0.0)
    , y(0.0)
    , w(0.0)
    , h(0.0)
    , angle(0.0)  {
    }
  RotatedRect_(const ContainerAllocator& _alloc)
    : is_exist(false)
    , x(0.0)
    , y(0.0)
    , w(0.0)
    , h(0.0)
    , angle(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _is_exist_type;
  _is_exist_type is_exist;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _w_type;
  _w_type w;

   typedef float _h_type;
  _h_type h;

   typedef float _angle_type;
  _angle_type angle;





  typedef boost::shared_ptr< ::vision::RotatedRect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision::RotatedRect_<ContainerAllocator> const> ConstPtr;

}; // struct RotatedRect_

typedef ::vision::RotatedRect_<std::allocator<void> > RotatedRect;

typedef boost::shared_ptr< ::vision::RotatedRect > RotatedRectPtr;
typedef boost::shared_ptr< ::vision::RotatedRect const> RotatedRectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision::RotatedRect_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision::RotatedRect_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vision::RotatedRect_<ContainerAllocator1> & lhs, const ::vision::RotatedRect_<ContainerAllocator2> & rhs)
{
  return lhs.is_exist == rhs.is_exist &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.w == rhs.w &&
    lhs.h == rhs.h &&
    lhs.angle == rhs.angle;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vision::RotatedRect_<ContainerAllocator1> & lhs, const ::vision::RotatedRect_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vision::RotatedRect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision::RotatedRect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::RotatedRect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision::RotatedRect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::RotatedRect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision::RotatedRect_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f115e26d1023013fc950dfc14ab1145";
  }

  static const char* value(const ::vision::RotatedRect_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f115e26d1023013ULL;
  static const uint64_t static_value2 = 0xfc950dfc14ab1145ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision/RotatedRect";
  }

  static const char* value(const ::vision::RotatedRect_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision::RotatedRect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool is_exist\n"
"float32 x\n"
"float32 y\n"
"float32 w\n"
"float32 h\n"
"float32 angle\n"
;
  }

  static const char* value(const ::vision::RotatedRect_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision::RotatedRect_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_exist);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.w);
      stream.next(m.h);
      stream.next(m.angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RotatedRect_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision::RotatedRect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision::RotatedRect_<ContainerAllocator>& v)
  {
    s << indent << "is_exist: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_exist);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "w: ";
    Printer<float>::stream(s, indent + "  ", v.w);
    s << indent << "h: ";
    Printer<float>::stream(s, indent + "  ", v.h);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MESSAGE_ROTATEDRECT_H