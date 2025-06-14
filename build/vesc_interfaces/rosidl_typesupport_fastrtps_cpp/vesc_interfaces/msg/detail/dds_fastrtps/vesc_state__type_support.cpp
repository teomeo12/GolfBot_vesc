// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vesc_interfaces:msg/VescState.idl
// generated code does not contain a copyright notice
#include "vesc_interfaces/msg/detail/vesc_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vesc_interfaces/msg/detail/vesc_state__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vesc_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vesc_interfaces
cdr_serialize(
  const vesc_interfaces::msg::VescState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: temp_fet
  cdr << ros_message.temp_fet;
  // Member: temp_motor
  cdr << ros_message.temp_motor;
  // Member: current_motor
  cdr << ros_message.current_motor;
  // Member: current_input
  cdr << ros_message.current_input;
  // Member: avg_id
  cdr << ros_message.avg_id;
  // Member: avg_iq
  cdr << ros_message.avg_iq;
  // Member: duty_cycle
  cdr << ros_message.duty_cycle;
  // Member: speed
  cdr << ros_message.speed;
  // Member: voltage_input
  cdr << ros_message.voltage_input;
  // Member: charge_drawn
  cdr << ros_message.charge_drawn;
  // Member: charge_regen
  cdr << ros_message.charge_regen;
  // Member: energy_drawn
  cdr << ros_message.energy_drawn;
  // Member: energy_regen
  cdr << ros_message.energy_regen;
  // Member: displacement
  cdr << ros_message.displacement;
  // Member: distance_traveled
  cdr << ros_message.distance_traveled;
  // Member: fault_code
  cdr << ros_message.fault_code;
  // Member: pid_pos_now
  cdr << ros_message.pid_pos_now;
  // Member: controller_id
  cdr << ros_message.controller_id;
  // Member: ntc_temp_mos1
  cdr << ros_message.ntc_temp_mos1;
  // Member: ntc_temp_mos2
  cdr << ros_message.ntc_temp_mos2;
  // Member: ntc_temp_mos3
  cdr << ros_message.ntc_temp_mos3;
  // Member: avg_vd
  cdr << ros_message.avg_vd;
  // Member: avg_vq
  cdr << ros_message.avg_vq;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vesc_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vesc_interfaces::msg::VescState & ros_message)
{
  // Member: temp_fet
  cdr >> ros_message.temp_fet;

  // Member: temp_motor
  cdr >> ros_message.temp_motor;

  // Member: current_motor
  cdr >> ros_message.current_motor;

  // Member: current_input
  cdr >> ros_message.current_input;

  // Member: avg_id
  cdr >> ros_message.avg_id;

  // Member: avg_iq
  cdr >> ros_message.avg_iq;

  // Member: duty_cycle
  cdr >> ros_message.duty_cycle;

  // Member: speed
  cdr >> ros_message.speed;

  // Member: voltage_input
  cdr >> ros_message.voltage_input;

  // Member: charge_drawn
  cdr >> ros_message.charge_drawn;

  // Member: charge_regen
  cdr >> ros_message.charge_regen;

  // Member: energy_drawn
  cdr >> ros_message.energy_drawn;

  // Member: energy_regen
  cdr >> ros_message.energy_regen;

  // Member: displacement
  cdr >> ros_message.displacement;

  // Member: distance_traveled
  cdr >> ros_message.distance_traveled;

  // Member: fault_code
  cdr >> ros_message.fault_code;

  // Member: pid_pos_now
  cdr >> ros_message.pid_pos_now;

  // Member: controller_id
  cdr >> ros_message.controller_id;

  // Member: ntc_temp_mos1
  cdr >> ros_message.ntc_temp_mos1;

  // Member: ntc_temp_mos2
  cdr >> ros_message.ntc_temp_mos2;

  // Member: ntc_temp_mos3
  cdr >> ros_message.ntc_temp_mos3;

  // Member: avg_vd
  cdr >> ros_message.avg_vd;

  // Member: avg_vq
  cdr >> ros_message.avg_vq;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vesc_interfaces
get_serialized_size(
  const vesc_interfaces::msg::VescState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: temp_fet
  {
    size_t item_size = sizeof(ros_message.temp_fet);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temp_motor
  {
    size_t item_size = sizeof(ros_message.temp_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_motor
  {
    size_t item_size = sizeof(ros_message.current_motor);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: current_input
  {
    size_t item_size = sizeof(ros_message.current_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: avg_id
  {
    size_t item_size = sizeof(ros_message.avg_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: avg_iq
  {
    size_t item_size = sizeof(ros_message.avg_iq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: duty_cycle
  {
    size_t item_size = sizeof(ros_message.duty_cycle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: voltage_input
  {
    size_t item_size = sizeof(ros_message.voltage_input);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: charge_drawn
  {
    size_t item_size = sizeof(ros_message.charge_drawn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: charge_regen
  {
    size_t item_size = sizeof(ros_message.charge_regen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: energy_drawn
  {
    size_t item_size = sizeof(ros_message.energy_drawn);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: energy_regen
  {
    size_t item_size = sizeof(ros_message.energy_regen);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: displacement
  {
    size_t item_size = sizeof(ros_message.displacement);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance_traveled
  {
    size_t item_size = sizeof(ros_message.distance_traveled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fault_code
  {
    size_t item_size = sizeof(ros_message.fault_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pid_pos_now
  {
    size_t item_size = sizeof(ros_message.pid_pos_now);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: controller_id
  {
    size_t item_size = sizeof(ros_message.controller_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ntc_temp_mos1
  {
    size_t item_size = sizeof(ros_message.ntc_temp_mos1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ntc_temp_mos2
  {
    size_t item_size = sizeof(ros_message.ntc_temp_mos2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ntc_temp_mos3
  {
    size_t item_size = sizeof(ros_message.ntc_temp_mos3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: avg_vd
  {
    size_t item_size = sizeof(ros_message.avg_vd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: avg_vq
  {
    size_t item_size = sizeof(ros_message.avg_vq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vesc_interfaces
max_serialized_size_VescState(
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


  // Member: temp_fet
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: temp_motor
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: current_motor
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: current_input
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: avg_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: avg_iq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: duty_cycle
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: voltage_input
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: charge_drawn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: charge_regen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: energy_drawn
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: energy_regen
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: displacement
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance_traveled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fault_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pid_pos_now
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: controller_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ntc_temp_mos1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ntc_temp_mos2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ntc_temp_mos3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: avg_vd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: avg_vq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vesc_interfaces::msg::VescState;
    is_plain =
      (
      offsetof(DataType, avg_vq) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _VescState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vesc_interfaces::msg::VescState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _VescState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vesc_interfaces::msg::VescState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _VescState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vesc_interfaces::msg::VescState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _VescState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_VescState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _VescState__callbacks = {
  "vesc_interfaces::msg",
  "VescState",
  _VescState__cdr_serialize,
  _VescState__cdr_deserialize,
  _VescState__get_serialized_size,
  _VescState__max_serialized_size
};

static rosidl_message_type_support_t _VescState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_VescState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vesc_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vesc_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<vesc_interfaces::msg::VescState>()
{
  return &vesc_interfaces::msg::typesupport_fastrtps_cpp::_VescState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vesc_interfaces, msg, VescState)() {
  return &vesc_interfaces::msg::typesupport_fastrtps_cpp::_VescState__handle;
}

#ifdef __cplusplus
}
#endif
