// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "aimrt_type_support_pkg_c_interface/type_support_pkg_main.h"

#include "aimrt_module_protobuf_interface/util/protobuf_type_support.h"
#include "imu.pb.h"
#include "joint_command.pb.h"
#include "joint_state.pb.h"
#include "touch_sensor_state.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include <sensor_msgs/msg/imu.hpp>
  #include "aimrt_module_ros2_interface/util/ros2_type_support.h"
  #include "sensor_ros2/msg/joint_command.hpp"
  #include "sensor_ros2/msg/joint_state.hpp"
  #include "sensor_ros2/msg/touch_sensor_state.hpp"
#endif

static const aimrt_type_support_base_t* type_support_array[]{
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::ImuState>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointState>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::JointCommand>(),
    aimrt::GetProtobufMessageTypeSupport<aimrt::protocols::sensor::TouchSensorState>(),

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
    aimrt::GetRos2MessageTypeSupport<sensor_msgs::msg::Imu>(),
    aimrt::GetRos2MessageTypeSupport<sensor_ros2::msg::JointState>(),
    aimrt::GetRos2MessageTypeSupport<sensor_ros2::msg::JointCommand>(),
    aimrt::GetRos2MessageTypeSupport<sensor_ros2::msg::TouchSensorState>(),
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