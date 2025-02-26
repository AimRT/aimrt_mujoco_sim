// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include <vector>

#include "joint_command.pb.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/subscriber/subscriber_base.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {

class JointActuatorSubscriber : public SubscriberBase {
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
  JointActuatorSubscriber() {}
  ~JointActuatorSubscriber() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override { stop_flag_ = false; }
  void Shutdown() override { stop_flag_ = true; }

  std::string_view Type() const noexcept override { return "joint_actuator"; }

  void SetMj(mjModel* m, mjData* d) override {
    m_ = m;
    d_ = d;
  }
  void SetSubscriberHandle(aimrt::channel::SubscriberRef subscriber_handle) override {
    subscriber_ = subscriber_handle;
  }

  void ApplyCtrlData() override;

 private:
  void EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointCommand>& commands);

  void RegisterActuatorAddr();

 private:
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

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber
