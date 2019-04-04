// Generated by gencpp from file dynamixel_workbench_msgs/DynamixelCommand.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMAND_H
#define DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMAND_H

#include <ros/service_traits.h>


#include <dynamixel_workbench_msgs/DynamixelCommandRequest.h>
#include <dynamixel_workbench_msgs/DynamixelCommandResponse.h>


namespace dynamixel_workbench_msgs
{

struct DynamixelCommand
{

typedef DynamixelCommandRequest Request;
typedef DynamixelCommandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DynamixelCommand
} // namespace dynamixel_workbench_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommand > {
  static const char* value()
  {
    return "cf973b38a14a7c815992bad0743e3f5b";
  }

  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommand&) { return value(); }
};

template<>
struct DataType< ::dynamixel_workbench_msgs::DynamixelCommand > {
  static const char* value()
  {
    return "dynamixel_workbench_msgs/DynamixelCommand";
  }

  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommand&) { return value(); }
};


// service_traits::MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommandRequest> should match 
// service_traits::MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommand > 
template<>
struct MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommand >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_workbench_msgs::DynamixelCommandRequest> should match 
// service_traits::DataType< ::dynamixel_workbench_msgs::DynamixelCommand > 
template<>
struct DataType< ::dynamixel_workbench_msgs::DynamixelCommandRequest>
{
  static const char* value()
  {
    return DataType< ::dynamixel_workbench_msgs::DynamixelCommand >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommandResponse> should match 
// service_traits::MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommand > 
template<>
struct MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommand >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_workbench_msgs::DynamixelCommandResponse> should match 
// service_traits::DataType< ::dynamixel_workbench_msgs::DynamixelCommand > 
template<>
struct DataType< ::dynamixel_workbench_msgs::DynamixelCommandResponse>
{
  static const char* value()
  {
    return DataType< ::dynamixel_workbench_msgs::DynamixelCommand >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMAND_H
