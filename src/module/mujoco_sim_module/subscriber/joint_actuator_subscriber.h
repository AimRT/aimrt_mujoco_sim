// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <vector>

#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "joint_command.pb.h"

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
  #include "aimrt_module_ros2_interface/channel/ros2_channel.h"
  #include "sensor_ros2/msg/joint_command.hpp"
#endif

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {
class JointActuatorSubscriberBase : public SubscriberBase {
 public:
  struct Options {
    struct Joint {
      std::string name;
      std::string bind_joint;
      std::string bind_actuator_type;
      std::string bind_actuator_name;
    };
    std::vector<Joint> joints;
  };

 public:
  JointActuatorSubscriberBase() = default;
  virtual ~JointActuatorSubscriberBase() = default;

  virtual void Initialize(YAML::Node options_node) = 0;
  virtual std::string_view Type() const noexcept = 0;

  void Start() override { stop_flag_ = false; }
  void Shutdown() override { stop_flag_ = true; }

  void SetMj(mjModel* m, mjData* d) override;
  void SetSubscriberHandle(aimrt::channel::SubscriberRef subscriber_handle) override { subscriber_ = subscriber_handle; }

  void ApplyCtrlData() override;

 protected:
  void InitializeBase(YAML::Node options_node);
  void RegisterActuatorAddr();

 protected:
  struct ActuatorBindJointSensorAddr {
    int32_t pos_addr;
    int32_t vel_addr;
  };

  Options options_;
  bool stop_flag_ = true;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  aimrt::channel::SubscriberRef subscriber_;

  size_t joint_num_ = 0;
  std::vector<size_t> actuator_addr_vec_;
  std::atomic<double*> command_array_{nullptr};
  std::vector<std::string> joint_names_vec_;

  std::vector<ActuatorBindJointSensorAddr> actuator_bind_joint_sensor_addr_vec_;
};

class JointActuatorSubscriber : public JointActuatorSubscriberBase {
 public:
  JointActuatorSubscriber() = default;
  ~JointActuatorSubscriber() override = default;

  void Initialize(YAML::Node options_node) override;
  std::string_view Type() const noexcept override { return "joint_actuator"; }

 private:
  void EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointCommand>& commands);
};

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
class JointActuatorRos2Subscriber : public JointActuatorSubscriberBase {
 public:
  JointActuatorRos2Subscriber() = default;
  ~JointActuatorRos2Subscriber() override = default;

  void Initialize(YAML::Node options_node) override;

  std::string_view Type() const noexcept override { return "joint_actuator_ros2"; }

 private:
  void EventHandle(const std::shared_ptr<const sensor_ros2::msg::JointCommand>& commands);
};
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
