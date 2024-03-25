// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "emo_voice_interfaces/srv/detail/gen_text__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace emo_voice_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GenText_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) emo_voice_interfaces::srv::GenText_Request(_init);
}

void GenText_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<emo_voice_interfaces::srv::GenText_Request *>(message_memory);
  typed_message->~GenText_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GenText_Request_message_member_array[2] = {
  {
    "emo",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(emo_voice_interfaces::srv::GenText_Request, emo),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(emo_voice_interfaces::srv::GenText_Request, name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GenText_Request_message_members = {
  "emo_voice_interfaces::srv",  // message namespace
  "GenText_Request",  // message name
  2,  // number of fields
  sizeof(emo_voice_interfaces::srv::GenText_Request),
  GenText_Request_message_member_array,  // message members
  GenText_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GenText_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GenText_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GenText_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace emo_voice_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<emo_voice_interfaces::srv::GenText_Request>()
{
  return &::emo_voice_interfaces::srv::rosidl_typesupport_introspection_cpp::GenText_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, emo_voice_interfaces, srv, GenText_Request)() {
  return &::emo_voice_interfaces::srv::rosidl_typesupport_introspection_cpp::GenText_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "emo_voice_interfaces/srv/detail/gen_text__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace emo_voice_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GenText_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) emo_voice_interfaces::srv::GenText_Response(_init);
}

void GenText_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<emo_voice_interfaces::srv::GenText_Response *>(message_memory);
  typed_message->~GenText_Response();
}

size_t size_function__GenText_Response__text(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GenText_Response__text(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__GenText_Response__text(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__GenText_Response__text(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__GenText_Response__text(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__GenText_Response__text(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__GenText_Response__text(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__GenText_Response__text(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GenText_Response_message_member_array[1] = {
  {
    "text",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(emo_voice_interfaces::srv::GenText_Response, text),  // bytes offset in struct
    nullptr,  // default value
    size_function__GenText_Response__text,  // size() function pointer
    get_const_function__GenText_Response__text,  // get_const(index) function pointer
    get_function__GenText_Response__text,  // get(index) function pointer
    fetch_function__GenText_Response__text,  // fetch(index, &value) function pointer
    assign_function__GenText_Response__text,  // assign(index, value) function pointer
    resize_function__GenText_Response__text  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GenText_Response_message_members = {
  "emo_voice_interfaces::srv",  // message namespace
  "GenText_Response",  // message name
  1,  // number of fields
  sizeof(emo_voice_interfaces::srv::GenText_Response),
  GenText_Response_message_member_array,  // message members
  GenText_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GenText_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GenText_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GenText_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace emo_voice_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<emo_voice_interfaces::srv::GenText_Response>()
{
  return &::emo_voice_interfaces::srv::rosidl_typesupport_introspection_cpp::GenText_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, emo_voice_interfaces, srv, GenText_Response)() {
  return &::emo_voice_interfaces::srv::rosidl_typesupport_introspection_cpp::GenText_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "emo_voice_interfaces/srv/detail/gen_text__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace emo_voice_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GenText_service_members = {
  "emo_voice_interfaces::srv",  // service namespace
  "GenText",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<emo_voice_interfaces::srv::GenText>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GenText_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GenText_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace emo_voice_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<emo_voice_interfaces::srv::GenText>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::emo_voice_interfaces::srv::rosidl_typesupport_introspection_cpp::GenText_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::emo_voice_interfaces::srv::GenText_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::emo_voice_interfaces::srv::GenText_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, emo_voice_interfaces, srv, GenText)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<emo_voice_interfaces::srv::GenText>();
}

#ifdef __cplusplus
}
#endif
