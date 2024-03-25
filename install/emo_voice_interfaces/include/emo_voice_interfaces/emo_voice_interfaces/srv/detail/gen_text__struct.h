// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#ifndef EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_H_
#define EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'emo'
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GenText in the package emo_voice_interfaces.
typedef struct emo_voice_interfaces__srv__GenText_Request
{
  rosidl_runtime_c__String emo;
  rosidl_runtime_c__String name;
} emo_voice_interfaces__srv__GenText_Request;

// Struct for a sequence of emo_voice_interfaces__srv__GenText_Request.
typedef struct emo_voice_interfaces__srv__GenText_Request__Sequence
{
  emo_voice_interfaces__srv__GenText_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} emo_voice_interfaces__srv__GenText_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GenText in the package emo_voice_interfaces.
typedef struct emo_voice_interfaces__srv__GenText_Response
{
  rosidl_runtime_c__int32__Sequence text;
} emo_voice_interfaces__srv__GenText_Response;

// Struct for a sequence of emo_voice_interfaces__srv__GenText_Response.
typedef struct emo_voice_interfaces__srv__GenText_Response__Sequence
{
  emo_voice_interfaces__srv__GenText_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} emo_voice_interfaces__srv__GenText_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_H_
