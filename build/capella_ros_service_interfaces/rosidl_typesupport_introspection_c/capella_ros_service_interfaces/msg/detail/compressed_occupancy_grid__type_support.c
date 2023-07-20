// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from capella_ros_service_interfaces:msg/CompressedOccupancyGrid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "capella_ros_service_interfaces/msg/detail/compressed_occupancy_grid__rosidl_typesupport_introspection_c.h"
#include "capella_ros_service_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "capella_ros_service_interfaces/msg/detail/compressed_occupancy_grid__functions.h"
#include "capella_ros_service_interfaces/msg/detail/compressed_occupancy_grid__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `info`
#include "nav_msgs/msg/map_meta_data.h"
// Member `info`
#include "nav_msgs/msg/detail/map_meta_data__rosidl_typesupport_introspection_c.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__init(message_memory);
}

void capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_fini_function(void * message_memory)
{
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__fini(message_memory);
}

size_t capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__size_function__CompressedOccupancyGrid__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_const_function__CompressedOccupancyGrid__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_function__CompressedOccupancyGrid__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__fetch_function__CompressedOccupancyGrid__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_const_function__CompressedOccupancyGrid__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__assign_function__CompressedOccupancyGrid__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_function__CompressedOccupancyGrid__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__resize_function__CompressedOccupancyGrid__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces__msg__CompressedOccupancyGrid, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces__msg__CompressedOccupancyGrid, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces__msg__CompressedOccupancyGrid, data),  // bytes offset in struct
    NULL,  // default value
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__size_function__CompressedOccupancyGrid__data,  // size() function pointer
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_const_function__CompressedOccupancyGrid__data,  // get_const(index) function pointer
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__get_function__CompressedOccupancyGrid__data,  // get(index) function pointer
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__fetch_function__CompressedOccupancyGrid__data,  // fetch(index, &value) function pointer
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__assign_function__CompressedOccupancyGrid__data,  // assign(index, value) function pointer
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__resize_function__CompressedOccupancyGrid__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_members = {
  "capella_ros_service_interfaces__msg",  // message namespace
  "CompressedOccupancyGrid",  // message name
  3,  // number of fields
  sizeof(capella_ros_service_interfaces__msg__CompressedOccupancyGrid),
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_member_array,  // message members
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_init_function,  // function to initialize message memory (memory has to be allocated)
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_type_support_handle = {
  0,
  &capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_capella_ros_service_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, capella_ros_service_interfaces, msg, CompressedOccupancyGrid)() {
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, nav_msgs, msg, MapMetaData)();
  if (!capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_type_support_handle.typesupport_identifier) {
    capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &capella_ros_service_interfaces__msg__CompressedOccupancyGrid__rosidl_typesupport_introspection_c__CompressedOccupancyGrid_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
