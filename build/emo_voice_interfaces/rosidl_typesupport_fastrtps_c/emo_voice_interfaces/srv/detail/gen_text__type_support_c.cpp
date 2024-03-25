// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice
#include "emo_voice_interfaces/srv/detail/gen_text__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "emo_voice_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "emo_voice_interfaces/srv/detail/gen_text__struct.h"
#include "emo_voice_interfaces/srv/detail/gen_text__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // emo, name
#include "rosidl_runtime_c/string_functions.h"  // emo, name

// forward declare type support functions


using _GenText_Request__ros_msg_type = emo_voice_interfaces__srv__GenText_Request;

static bool _GenText_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GenText_Request__ros_msg_type * ros_message = static_cast<const _GenText_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: emo
  {
    const rosidl_runtime_c__String * str = &ros_message->emo;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: name
  {
    const rosidl_runtime_c__String * str = &ros_message->name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _GenText_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GenText_Request__ros_msg_type * ros_message = static_cast<_GenText_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: emo
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->emo.data) {
      rosidl_runtime_c__String__init(&ros_message->emo);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->emo,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'emo'\n");
      return false;
    }
  }

  // Field name: name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->name.data) {
      rosidl_runtime_c__String__init(&ros_message->name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'name'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_emo_voice_interfaces
size_t get_serialized_size_emo_voice_interfaces__srv__GenText_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GenText_Request__ros_msg_type * ros_message = static_cast<const _GenText_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name emo
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->emo.size + 1);
  // field.name name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->name.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _GenText_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_emo_voice_interfaces__srv__GenText_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_emo_voice_interfaces
size_t max_serialized_size_emo_voice_interfaces__srv__GenText_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: emo
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = emo_voice_interfaces__srv__GenText_Request;
    is_plain =
      (
      offsetof(DataType, name) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GenText_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_emo_voice_interfaces__srv__GenText_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GenText_Request = {
  "emo_voice_interfaces::srv",
  "GenText_Request",
  _GenText_Request__cdr_serialize,
  _GenText_Request__cdr_deserialize,
  _GenText_Request__get_serialized_size,
  _GenText_Request__max_serialized_size
};

static rosidl_message_type_support_t _GenText_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GenText_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, emo_voice_interfaces, srv, GenText_Request)() {
  return &_GenText_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "emo_voice_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "emo_voice_interfaces/srv/detail/gen_text__struct.h"
// already included above
// #include "emo_voice_interfaces/srv/detail/gen_text__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // text
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // text

// forward declare type support functions


using _GenText_Response__ros_msg_type = emo_voice_interfaces__srv__GenText_Response;

static bool _GenText_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GenText_Response__ros_msg_type * ros_message = static_cast<const _GenText_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: text
  {
    size_t size = ros_message->text.size;
    auto array_ptr = ros_message->text.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _GenText_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GenText_Response__ros_msg_type * ros_message = static_cast<_GenText_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: text
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->text.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->text);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->text, size)) {
      fprintf(stderr, "failed to create array for field 'text'");
      return false;
    }
    auto array_ptr = ros_message->text.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_emo_voice_interfaces
size_t get_serialized_size_emo_voice_interfaces__srv__GenText_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GenText_Response__ros_msg_type * ros_message = static_cast<const _GenText_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name text
  {
    size_t array_size = ros_message->text.size;
    auto array_ptr = ros_message->text.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GenText_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_emo_voice_interfaces__srv__GenText_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_emo_voice_interfaces
size_t max_serialized_size_emo_voice_interfaces__srv__GenText_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: text
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = emo_voice_interfaces__srv__GenText_Response;
    is_plain =
      (
      offsetof(DataType, text) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GenText_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_emo_voice_interfaces__srv__GenText_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GenText_Response = {
  "emo_voice_interfaces::srv",
  "GenText_Response",
  _GenText_Response__cdr_serialize,
  _GenText_Response__cdr_deserialize,
  _GenText_Response__get_serialized_size,
  _GenText_Response__max_serialized_size
};

static rosidl_message_type_support_t _GenText_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GenText_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, emo_voice_interfaces, srv, GenText_Response)() {
  return &_GenText_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "emo_voice_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "emo_voice_interfaces/srv/gen_text.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GenText__callbacks = {
  "emo_voice_interfaces::srv",
  "GenText",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, emo_voice_interfaces, srv, GenText_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, emo_voice_interfaces, srv, GenText_Response)(),
};

static rosidl_service_type_support_t GenText__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GenText__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, emo_voice_interfaces, srv, GenText)() {
  return &GenText__handle;
}

#if defined(__cplusplus)
}
#endif
