// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#ifndef EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__BUILDER_HPP_
#define EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "emo_voice_interfaces/srv/detail/gen_text__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace emo_voice_interfaces
{

namespace srv
{

namespace builder
{

class Init_GenText_Request_name
{
public:
  explicit Init_GenText_Request_name(::emo_voice_interfaces::srv::GenText_Request & msg)
  : msg_(msg)
  {}
  ::emo_voice_interfaces::srv::GenText_Request name(::emo_voice_interfaces::srv::GenText_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::emo_voice_interfaces::srv::GenText_Request msg_;
};

class Init_GenText_Request_emo
{
public:
  Init_GenText_Request_emo()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GenText_Request_name emo(::emo_voice_interfaces::srv::GenText_Request::_emo_type arg)
  {
    msg_.emo = std::move(arg);
    return Init_GenText_Request_name(msg_);
  }

private:
  ::emo_voice_interfaces::srv::GenText_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::emo_voice_interfaces::srv::GenText_Request>()
{
  return emo_voice_interfaces::srv::builder::Init_GenText_Request_emo();
}

}  // namespace emo_voice_interfaces


namespace emo_voice_interfaces
{

namespace srv
{

namespace builder
{

class Init_GenText_Response_text
{
public:
  Init_GenText_Response_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::emo_voice_interfaces::srv::GenText_Response text(::emo_voice_interfaces::srv::GenText_Response::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::emo_voice_interfaces::srv::GenText_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::emo_voice_interfaces::srv::GenText_Response>()
{
  return emo_voice_interfaces::srv::builder::Init_GenText_Response_text();
}

}  // namespace emo_voice_interfaces

#endif  // EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__BUILDER_HPP_
