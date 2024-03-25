// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#ifndef EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_HPP_
#define EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__emo_voice_interfaces__srv__GenText_Request __attribute__((deprecated))
#else
# define DEPRECATED__emo_voice_interfaces__srv__GenText_Request __declspec(deprecated)
#endif

namespace emo_voice_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenText_Request_
{
  using Type = GenText_Request_<ContainerAllocator>;

  explicit GenText_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emo = "";
      this->name = "";
    }
  }

  explicit GenText_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : emo(_alloc),
    name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->emo = "";
      this->name = "";
    }
  }

  // field types and members
  using _emo_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _emo_type emo;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__emo(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->emo = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__emo_voice_interfaces__srv__GenText_Request
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__emo_voice_interfaces__srv__GenText_Request
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenText_Request_ & other) const
  {
    if (this->emo != other.emo) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenText_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenText_Request_

// alias to use template instance with default allocator
using GenText_Request =
  emo_voice_interfaces::srv::GenText_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace emo_voice_interfaces


#ifndef _WIN32
# define DEPRECATED__emo_voice_interfaces__srv__GenText_Response __attribute__((deprecated))
#else
# define DEPRECATED__emo_voice_interfaces__srv__GenText_Response __declspec(deprecated)
#endif

namespace emo_voice_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenText_Response_
{
  using Type = GenText_Response_<ContainerAllocator>;

  explicit GenText_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GenText_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _text_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _text_type text;

  // setters for named parameter idiom
  Type & set__text(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->text = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__emo_voice_interfaces__srv__GenText_Response
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__emo_voice_interfaces__srv__GenText_Response
    std::shared_ptr<emo_voice_interfaces::srv::GenText_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenText_Response_ & other) const
  {
    if (this->text != other.text) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenText_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenText_Response_

// alias to use template instance with default allocator
using GenText_Response =
  emo_voice_interfaces::srv::GenText_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace emo_voice_interfaces

namespace emo_voice_interfaces
{

namespace srv
{

struct GenText
{
  using Request = emo_voice_interfaces::srv::GenText_Request;
  using Response = emo_voice_interfaces::srv::GenText_Response;
};

}  // namespace srv

}  // namespace emo_voice_interfaces

#endif  // EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__STRUCT_HPP_
