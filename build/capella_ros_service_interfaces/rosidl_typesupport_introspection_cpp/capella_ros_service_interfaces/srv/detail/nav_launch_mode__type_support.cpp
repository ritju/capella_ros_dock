// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from capella_ros_service_interfaces:srv/NavLaunchMode.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "capella_ros_service_interfaces/srv/detail/nav_launch_mode__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace capella_ros_service_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void NavLaunchMode_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) capella_ros_service_interfaces::srv::NavLaunchMode_Request(_init);
}

void NavLaunchMode_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<capella_ros_service_interfaces::srv::NavLaunchMode_Request *>(message_memory);
  typed_message->~NavLaunchMode_Request();
}

size_t size_function__NavLaunchMode_Request__image(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NavLaunchMode_Request__image(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__NavLaunchMode_Request__image(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__NavLaunchMode_Request__image(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__NavLaunchMode_Request__image(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__NavLaunchMode_Request__image(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__NavLaunchMode_Request__image(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__NavLaunchMode_Request__image(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NavLaunchMode_Request_message_member_array[8] = {
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, image),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavLaunchMode_Request__image,  // size() function pointer
    get_const_function__NavLaunchMode_Request__image,  // get_const(index) function pointer
    get_function__NavLaunchMode_Request__image,  // get(index) function pointer
    fetch_function__NavLaunchMode_Request__image,  // fetch(index, &value) function pointer
    assign_function__NavLaunchMode_Request__image,  // assign(index, value) function pointer
    resize_function__NavLaunchMode_Request__image  // resize(index) function pointer
  },
  {
    "resolution",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, resolution),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, z),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "negate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, negate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "occupied_thresh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, occupied_thresh),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "free_thresh",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Request, free_thresh),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NavLaunchMode_Request_message_members = {
  "capella_ros_service_interfaces::srv",  // message namespace
  "NavLaunchMode_Request",  // message name
  8,  // number of fields
  sizeof(capella_ros_service_interfaces::srv::NavLaunchMode_Request),
  NavLaunchMode_Request_message_member_array,  // message members
  NavLaunchMode_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  NavLaunchMode_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NavLaunchMode_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavLaunchMode_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace capella_ros_service_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<capella_ros_service_interfaces::srv::NavLaunchMode_Request>()
{
  return &::capella_ros_service_interfaces::srv::rosidl_typesupport_introspection_cpp::NavLaunchMode_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, capella_ros_service_interfaces, srv, NavLaunchMode_Request)() {
  return &::capella_ros_service_interfaces::srv::rosidl_typesupport_introspection_cpp::NavLaunchMode_Request_message_type_support_handle;
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
// #include "capella_ros_service_interfaces/srv/detail/nav_launch_mode__struct.hpp"
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

namespace capella_ros_service_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void NavLaunchMode_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) capella_ros_service_interfaces::srv::NavLaunchMode_Response(_init);
}

void NavLaunchMode_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<capella_ros_service_interfaces::srv::NavLaunchMode_Response *>(message_memory);
  typed_message->~NavLaunchMode_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NavLaunchMode_Response_message_member_array[1] = {
  {
    "nav_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(capella_ros_service_interfaces::srv::NavLaunchMode_Response, nav_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NavLaunchMode_Response_message_members = {
  "capella_ros_service_interfaces::srv",  // message namespace
  "NavLaunchMode_Response",  // message name
  1,  // number of fields
  sizeof(capella_ros_service_interfaces::srv::NavLaunchMode_Response),
  NavLaunchMode_Response_message_member_array,  // message members
  NavLaunchMode_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  NavLaunchMode_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NavLaunchMode_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavLaunchMode_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace capella_ros_service_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<capella_ros_service_interfaces::srv::NavLaunchMode_Response>()
{
  return &::capella_ros_service_interfaces::srv::rosidl_typesupport_introspection_cpp::NavLaunchMode_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, capella_ros_service_interfaces, srv, NavLaunchMode_Response)() {
  return &::capella_ros_service_interfaces::srv::rosidl_typesupport_introspection_cpp::NavLaunchMode_Response_message_type_support_handle;
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
// #include "capella_ros_service_interfaces/srv/detail/nav_launch_mode__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace capella_ros_service_interfaces
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers NavLaunchMode_service_members = {
  "capella_ros_service_interfaces::srv",  // service namespace
  "NavLaunchMode",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<capella_ros_service_interfaces::srv::NavLaunchMode>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t NavLaunchMode_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavLaunchMode_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace capella_ros_service_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<capella_ros_service_interfaces::srv::NavLaunchMode>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::capella_ros_service_interfaces::srv::rosidl_typesupport_introspection_cpp::NavLaunchMode_service_type_support_handle;
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
        ::capella_ros_service_interfaces::srv::NavLaunchMode_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::capella_ros_service_interfaces::srv::NavLaunchMode_Response
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, capella_ros_service_interfaces, srv, NavLaunchMode)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<capella_ros_service_interfaces::srv::NavLaunchMode>();
}

#ifdef __cplusplus
}
#endif
