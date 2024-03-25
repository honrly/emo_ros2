// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from emo_voice_interfaces:srv/GenText.idl
// generated code does not contain a copyright notice

#ifndef EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__TRAITS_HPP_
#define EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "emo_voice_interfaces/srv/detail/gen_text__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace emo_voice_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenText_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: emo
  {
    out << "emo: ";
    rosidl_generator_traits::value_to_yaml(msg.emo, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenText_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: emo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emo: ";
    rosidl_generator_traits::value_to_yaml(msg.emo, out);
    out << "\n";
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenText_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace emo_voice_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use emo_voice_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const emo_voice_interfaces::srv::GenText_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  emo_voice_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use emo_voice_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const emo_voice_interfaces::srv::GenText_Request & msg)
{
  return emo_voice_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<emo_voice_interfaces::srv::GenText_Request>()
{
  return "emo_voice_interfaces::srv::GenText_Request";
}

template<>
inline const char * name<emo_voice_interfaces::srv::GenText_Request>()
{
  return "emo_voice_interfaces/srv/GenText_Request";
}

template<>
struct has_fixed_size<emo_voice_interfaces::srv::GenText_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<emo_voice_interfaces::srv::GenText_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<emo_voice_interfaces::srv::GenText_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace emo_voice_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenText_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: text
  {
    if (msg.text.size() == 0) {
      out << "text: []";
    } else {
      out << "text: [";
      size_t pending_items = msg.text.size();
      for (auto item : msg.text) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenText_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: text
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.text.size() == 0) {
      out << "text: []\n";
    } else {
      out << "text:\n";
      for (auto item : msg.text) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenText_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace emo_voice_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use emo_voice_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const emo_voice_interfaces::srv::GenText_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  emo_voice_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use emo_voice_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const emo_voice_interfaces::srv::GenText_Response & msg)
{
  return emo_voice_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<emo_voice_interfaces::srv::GenText_Response>()
{
  return "emo_voice_interfaces::srv::GenText_Response";
}

template<>
inline const char * name<emo_voice_interfaces::srv::GenText_Response>()
{
  return "emo_voice_interfaces/srv/GenText_Response";
}

template<>
struct has_fixed_size<emo_voice_interfaces::srv::GenText_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<emo_voice_interfaces::srv::GenText_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<emo_voice_interfaces::srv::GenText_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<emo_voice_interfaces::srv::GenText>()
{
  return "emo_voice_interfaces::srv::GenText";
}

template<>
inline const char * name<emo_voice_interfaces::srv::GenText>()
{
  return "emo_voice_interfaces/srv/GenText";
}

template<>
struct has_fixed_size<emo_voice_interfaces::srv::GenText>
  : std::integral_constant<
    bool,
    has_fixed_size<emo_voice_interfaces::srv::GenText_Request>::value &&
    has_fixed_size<emo_voice_interfaces::srv::GenText_Response>::value
  >
{
};

template<>
struct has_bounded_size<emo_voice_interfaces::srv::GenText>
  : std::integral_constant<
    bool,
    has_bounded_size<emo_voice_interfaces::srv::GenText_Request>::value &&
    has_bounded_size<emo_voice_interfaces::srv::GenText_Response>::value
  >
{
};

template<>
struct is_service<emo_voice_interfaces::srv::GenText>
  : std::true_type
{
};

template<>
struct is_service_request<emo_voice_interfaces::srv::GenText_Request>
  : std::true_type
{
};

template<>
struct is_service_response<emo_voice_interfaces::srv::GenText_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EMO_VOICE_INTERFACES__SRV__DETAIL__GEN_TEXT__TRAITS_HPP_
