// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice
#include "emo_voice_interfaces/srv/detail/gen_text__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `emo`
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
emo_voice_interfaces__srv__GenText_Request__init(emo_voice_interfaces__srv__GenText_Request * msg)
{
  if (!msg) {
    return false;
  }
  // emo
  if (!rosidl_runtime_c__String__init(&msg->emo)) {
    emo_voice_interfaces__srv__GenText_Request__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    emo_voice_interfaces__srv__GenText_Request__fini(msg);
    return false;
  }
  return true;
}

void
emo_voice_interfaces__srv__GenText_Request__fini(emo_voice_interfaces__srv__GenText_Request * msg)
{
  if (!msg) {
    return;
  }
  // emo
  rosidl_runtime_c__String__fini(&msg->emo);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
}

bool
emo_voice_interfaces__srv__GenText_Request__are_equal(const emo_voice_interfaces__srv__GenText_Request * lhs, const emo_voice_interfaces__srv__GenText_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // emo
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->emo), &(rhs->emo)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  return true;
}

bool
emo_voice_interfaces__srv__GenText_Request__copy(
  const emo_voice_interfaces__srv__GenText_Request * input,
  emo_voice_interfaces__srv__GenText_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // emo
  if (!rosidl_runtime_c__String__copy(
      &(input->emo), &(output->emo)))
  {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  return true;
}

emo_voice_interfaces__srv__GenText_Request *
emo_voice_interfaces__srv__GenText_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Request * msg = (emo_voice_interfaces__srv__GenText_Request *)allocator.allocate(sizeof(emo_voice_interfaces__srv__GenText_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(emo_voice_interfaces__srv__GenText_Request));
  bool success = emo_voice_interfaces__srv__GenText_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
emo_voice_interfaces__srv__GenText_Request__destroy(emo_voice_interfaces__srv__GenText_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    emo_voice_interfaces__srv__GenText_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
emo_voice_interfaces__srv__GenText_Request__Sequence__init(emo_voice_interfaces__srv__GenText_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Request * data = NULL;

  if (size) {
    data = (emo_voice_interfaces__srv__GenText_Request *)allocator.zero_allocate(size, sizeof(emo_voice_interfaces__srv__GenText_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = emo_voice_interfaces__srv__GenText_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        emo_voice_interfaces__srv__GenText_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
emo_voice_interfaces__srv__GenText_Request__Sequence__fini(emo_voice_interfaces__srv__GenText_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      emo_voice_interfaces__srv__GenText_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

emo_voice_interfaces__srv__GenText_Request__Sequence *
emo_voice_interfaces__srv__GenText_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Request__Sequence * array = (emo_voice_interfaces__srv__GenText_Request__Sequence *)allocator.allocate(sizeof(emo_voice_interfaces__srv__GenText_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = emo_voice_interfaces__srv__GenText_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
emo_voice_interfaces__srv__GenText_Request__Sequence__destroy(emo_voice_interfaces__srv__GenText_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    emo_voice_interfaces__srv__GenText_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
emo_voice_interfaces__srv__GenText_Request__Sequence__are_equal(const emo_voice_interfaces__srv__GenText_Request__Sequence * lhs, const emo_voice_interfaces__srv__GenText_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!emo_voice_interfaces__srv__GenText_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
emo_voice_interfaces__srv__GenText_Request__Sequence__copy(
  const emo_voice_interfaces__srv__GenText_Request__Sequence * input,
  emo_voice_interfaces__srv__GenText_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(emo_voice_interfaces__srv__GenText_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    emo_voice_interfaces__srv__GenText_Request * data =
      (emo_voice_interfaces__srv__GenText_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!emo_voice_interfaces__srv__GenText_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          emo_voice_interfaces__srv__GenText_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!emo_voice_interfaces__srv__GenText_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `text`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
emo_voice_interfaces__srv__GenText_Response__init(emo_voice_interfaces__srv__GenText_Response * msg)
{
  if (!msg) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->text, 0)) {
    emo_voice_interfaces__srv__GenText_Response__fini(msg);
    return false;
  }
  return true;
}

void
emo_voice_interfaces__srv__GenText_Response__fini(emo_voice_interfaces__srv__GenText_Response * msg)
{
  if (!msg) {
    return;
  }
  // text
  rosidl_runtime_c__int32__Sequence__fini(&msg->text);
}

bool
emo_voice_interfaces__srv__GenText_Response__are_equal(const emo_voice_interfaces__srv__GenText_Response * lhs, const emo_voice_interfaces__srv__GenText_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->text), &(rhs->text)))
  {
    return false;
  }
  return true;
}

bool
emo_voice_interfaces__srv__GenText_Response__copy(
  const emo_voice_interfaces__srv__GenText_Response * input,
  emo_voice_interfaces__srv__GenText_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // text
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->text), &(output->text)))
  {
    return false;
  }
  return true;
}

emo_voice_interfaces__srv__GenText_Response *
emo_voice_interfaces__srv__GenText_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Response * msg = (emo_voice_interfaces__srv__GenText_Response *)allocator.allocate(sizeof(emo_voice_interfaces__srv__GenText_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(emo_voice_interfaces__srv__GenText_Response));
  bool success = emo_voice_interfaces__srv__GenText_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
emo_voice_interfaces__srv__GenText_Response__destroy(emo_voice_interfaces__srv__GenText_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    emo_voice_interfaces__srv__GenText_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
emo_voice_interfaces__srv__GenText_Response__Sequence__init(emo_voice_interfaces__srv__GenText_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Response * data = NULL;

  if (size) {
    data = (emo_voice_interfaces__srv__GenText_Response *)allocator.zero_allocate(size, sizeof(emo_voice_interfaces__srv__GenText_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = emo_voice_interfaces__srv__GenText_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        emo_voice_interfaces__srv__GenText_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
emo_voice_interfaces__srv__GenText_Response__Sequence__fini(emo_voice_interfaces__srv__GenText_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      emo_voice_interfaces__srv__GenText_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

emo_voice_interfaces__srv__GenText_Response__Sequence *
emo_voice_interfaces__srv__GenText_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  emo_voice_interfaces__srv__GenText_Response__Sequence * array = (emo_voice_interfaces__srv__GenText_Response__Sequence *)allocator.allocate(sizeof(emo_voice_interfaces__srv__GenText_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = emo_voice_interfaces__srv__GenText_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
emo_voice_interfaces__srv__GenText_Response__Sequence__destroy(emo_voice_interfaces__srv__GenText_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    emo_voice_interfaces__srv__GenText_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
emo_voice_interfaces__srv__GenText_Response__Sequence__are_equal(const emo_voice_interfaces__srv__GenText_Response__Sequence * lhs, const emo_voice_interfaces__srv__GenText_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!emo_voice_interfaces__srv__GenText_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
emo_voice_interfaces__srv__GenText_Response__Sequence__copy(
  const emo_voice_interfaces__srv__GenText_Response__Sequence * input,
  emo_voice_interfaces__srv__GenText_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(emo_voice_interfaces__srv__GenText_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    emo_voice_interfaces__srv__GenText_Response * data =
      (emo_voice_interfaces__srv__GenText_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!emo_voice_interfaces__srv__GenText_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          emo_voice_interfaces__srv__GenText_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!emo_voice_interfaces__srv__GenText_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
