// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_protobuf_interface/util/protobuf_type_support.h"

#include "joint_command.pb.h"
#include "joint_state.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "aimrt_module_ros2_interface/util/ros2_type_support.h"
  #include "sensor_ros2/msg/joint_command.hpp"
  #include "sensor_ros2/msg/joint_state.hpp"
#endif

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointState>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointCommand>(),
#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
    aimrt::GetRos2MessageTypeSupport<sensor_ros2::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<sensor_ros2::msg::JointCommand>(),
#endif

};

extern "C" {

size_t AimRTDynlibGetTypeSupportArrayLength() {
  return sizeof(type_support_array) / sizeof(type_support_array[0]);
}

const aimrt_type_support_base_t** AimRTDynlibGetTypeSupportArray() {
  return type_support_array;
}
}