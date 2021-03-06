// Generated by gencpp from file object_msgs/RegisterObject.msg
// DO NOT EDIT!


#ifndef OBJECT_MSGS_MESSAGE_REGISTEROBJECT_H
#define OBJECT_MSGS_MESSAGE_REGISTEROBJECT_H

#include <ros/service_traits.h>


#include <object_msgs/RegisterObjectRequest.h>
#include <object_msgs/RegisterObjectResponse.h>


namespace object_msgs
{

struct RegisterObject
{

typedef RegisterObjectRequest Request;
typedef RegisterObjectResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RegisterObject
} // namespace object_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::object_msgs::RegisterObject > {
  static const char* value()
  {
    return "325b0c2df13037cc771f5f0cd7f14976";
  }

  static const char* value(const ::object_msgs::RegisterObject&) { return value(); }
};

template<>
struct DataType< ::object_msgs::RegisterObject > {
  static const char* value()
  {
    return "object_msgs/RegisterObject";
  }

  static const char* value(const ::object_msgs::RegisterObject&) { return value(); }
};


// service_traits::MD5Sum< ::object_msgs::RegisterObjectRequest> should match
// service_traits::MD5Sum< ::object_msgs::RegisterObject >
template<>
struct MD5Sum< ::object_msgs::RegisterObjectRequest>
{
  static const char* value()
  {
    return MD5Sum< ::object_msgs::RegisterObject >::value();
  }
  static const char* value(const ::object_msgs::RegisterObjectRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::object_msgs::RegisterObjectRequest> should match
// service_traits::DataType< ::object_msgs::RegisterObject >
template<>
struct DataType< ::object_msgs::RegisterObjectRequest>
{
  static const char* value()
  {
    return DataType< ::object_msgs::RegisterObject >::value();
  }
  static const char* value(const ::object_msgs::RegisterObjectRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::object_msgs::RegisterObjectResponse> should match
// service_traits::MD5Sum< ::object_msgs::RegisterObject >
template<>
struct MD5Sum< ::object_msgs::RegisterObjectResponse>
{
  static const char* value()
  {
    return MD5Sum< ::object_msgs::RegisterObject >::value();
  }
  static const char* value(const ::object_msgs::RegisterObjectResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::object_msgs::RegisterObjectResponse> should match
// service_traits::DataType< ::object_msgs::RegisterObject >
template<>
struct DataType< ::object_msgs::RegisterObjectResponse>
{
  static const char* value()
  {
    return DataType< ::object_msgs::RegisterObject >::value();
  }
  static const char* value(const ::object_msgs::RegisterObjectResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OBJECT_MSGS_MESSAGE_REGISTEROBJECT_H
