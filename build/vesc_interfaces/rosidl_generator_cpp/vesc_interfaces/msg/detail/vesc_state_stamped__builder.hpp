// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vesc_interfaces:msg/VescStateStamped.idl
// generated code does not contain a copyright notice

#ifndef VESC_INTERFACES__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_
#define VESC_INTERFACES__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vesc_interfaces/msg/detail/vesc_state_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vesc_interfaces
{

namespace msg
{

namespace builder
{

class Init_VescStateStamped_state
{
public:
  explicit Init_VescStateStamped_state(::vesc_interfaces::msg::VescStateStamped & msg)
  : msg_(msg)
  {}
  ::vesc_interfaces::msg::VescStateStamped state(::vesc_interfaces::msg::VescStateStamped::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vesc_interfaces::msg::VescStateStamped msg_;
};

class Init_VescStateStamped_header
{
public:
  Init_VescStateStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VescStateStamped_state header(::vesc_interfaces::msg::VescStateStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VescStateStamped_state(msg_);
  }

private:
  ::vesc_interfaces::msg::VescStateStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vesc_interfaces::msg::VescStateStamped>()
{
  return vesc_interfaces::msg::builder::Init_VescStateStamped_header();
}

}  // namespace vesc_interfaces

#endif  // VESC_INTERFACES__MSG__DETAIL__VESC_STATE_STAMPED__BUILDER_HPP_
