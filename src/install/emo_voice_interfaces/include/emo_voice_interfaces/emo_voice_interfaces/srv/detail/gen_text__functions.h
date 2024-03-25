// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#ifndef EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__FUNCTIONS_H_
#define EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "emo_voice_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "emo_voice_interfaces/srv/detail/gen_text__struct.h"

/// Initialize srv/GenText message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * emo_voice_interfaces__srv__GenText_Request
 * )) before or use
 * emo_voice_interfaces__srv__GenText_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__init(emo_voice_interfaces__srv__GenText_Request * msg);

/// Finalize srv/GenText message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Request__fini(emo_voice_interfaces__srv__GenText_Request * msg);

/// Create srv/GenText message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * emo_voice_interfaces__srv__GenText_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
emo_voice_interfaces__srv__GenText_Request *
emo_voice_interfaces__srv__GenText_Request__create();

/// Destroy srv/GenText message.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Request__destroy(emo_voice_interfaces__srv__GenText_Request * msg);

/// Check for srv/GenText message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__are_equal(const emo_voice_interfaces__srv__GenText_Request * lhs, const emo_voice_interfaces__srv__GenText_Request * rhs);

/// Copy a srv/GenText message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__copy(
  const emo_voice_interfaces__srv__GenText_Request * input,
  emo_voice_interfaces__srv__GenText_Request * output);

/// Initialize array of srv/GenText messages.
/**
 * It allocates the memory for the number of elements and calls
 * emo_voice_interfaces__srv__GenText_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__Sequence__init(emo_voice_interfaces__srv__GenText_Request__Sequence * array, size_t size);

/// Finalize array of srv/GenText messages.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Request__Sequence__fini(emo_voice_interfaces__srv__GenText_Request__Sequence * array);

/// Create array of srv/GenText messages.
/**
 * It allocates the memory for the array and calls
 * emo_voice_interfaces__srv__GenText_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
emo_voice_interfaces__srv__GenText_Request__Sequence *
emo_voice_interfaces__srv__GenText_Request__Sequence__create(size_t size);

/// Destroy array of srv/GenText messages.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Request__Sequence__destroy(emo_voice_interfaces__srv__GenText_Request__Sequence * array);

/// Check for srv/GenText message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__Sequence__are_equal(const emo_voice_interfaces__srv__GenText_Request__Sequence * lhs, const emo_voice_interfaces__srv__GenText_Request__Sequence * rhs);

/// Copy an array of srv/GenText messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Request__Sequence__copy(
  const emo_voice_interfaces__srv__GenText_Request__Sequence * input,
  emo_voice_interfaces__srv__GenText_Request__Sequence * output);

/// Initialize srv/GenText message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * emo_voice_interfaces__srv__GenText_Response
 * )) before or use
 * emo_voice_interfaces__srv__GenText_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__init(emo_voice_interfaces__srv__GenText_Response * msg);

/// Finalize srv/GenText message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Response__fini(emo_voice_interfaces__srv__GenText_Response * msg);

/// Create srv/GenText message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * emo_voice_interfaces__srv__GenText_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
emo_voice_interfaces__srv__GenText_Response *
emo_voice_interfaces__srv__GenText_Response__create();

/// Destroy srv/GenText message.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Response__destroy(emo_voice_interfaces__srv__GenText_Response * msg);

/// Check for srv/GenText message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__are_equal(const emo_voice_interfaces__srv__GenText_Response * lhs, const emo_voice_interfaces__srv__GenText_Response * rhs);

/// Copy a srv/GenText message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__copy(
  const emo_voice_interfaces__srv__GenText_Response * input,
  emo_voice_interfaces__srv__GenText_Response * output);

/// Initialize array of srv/GenText messages.
/**
 * It allocates the memory for the number of elements and calls
 * emo_voice_interfaces__srv__GenText_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__Sequence__init(emo_voice_interfaces__srv__GenText_Response__Sequence * array, size_t size);

/// Finalize array of srv/GenText messages.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Response__Sequence__fini(emo_voice_interfaces__srv__GenText_Response__Sequence * array);

/// Create array of srv/GenText messages.
/**
 * It allocates the memory for the array and calls
 * emo_voice_interfaces__srv__GenText_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
emo_voice_interfaces__srv__GenText_Response__Sequence *
emo_voice_interfaces__srv__GenText_Response__Sequence__create(size_t size);

/// Destroy array of srv/GenText messages.
/**
 * It calls
 * emo_voice_interfaces__srv__GenText_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
void
emo_voice_interfaces__srv__GenText_Response__Sequence__destroy(emo_voice_interfaces__srv__GenText_Response__Sequence * array);

/// Check for srv/GenText message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__Sequence__are_equal(const emo_voice_interfaces__srv__GenText_Response__Sequence * lhs, const emo_voice_interfaces__srv__GenText_Response__Sequence * rhs);

/// Copy an array of srv/GenText messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_emo_voice_interfaces
bool
emo_voice_interfaces__srv__GenText_Response__Sequence__copy(
  const emo_voice_interfaces__srv__GenText_Response__Sequence * input,
  emo_voice_interfaces__srv__GenText_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__FUNCTIONS_H_
