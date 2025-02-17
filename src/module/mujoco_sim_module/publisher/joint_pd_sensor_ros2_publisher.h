// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"
#include "sensor_ros2/msg/joint_pd_state.hpp"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

class JointPdSensorRos2Publisher : public PublisherBase {
 public:
  struct Options {
    struct Joint {
      std::string name;
      std::string bind_joint;
      std::string bind_jointpos_sensor;
      std::string bind_jointvel_sensor;
      std::string bind_jointactuatorfrc_sensor;
    };
    std::vector<Joint> joints;
  };

 public:
  JointPdSensorRos2Publisher() = default;
  ~JointPdSensorRos2Publisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override {}
  void Shutdown() override {}

  [[nodiscard]] std::string_view Type() const noexcept override { return "joint_pd_sensor_ros2"; }

  void SetPublisherHandle(aimrt::channel::PublisherRef publisher_handle) override {
    publisher_ = publisher_handle;
  }

  void SetMj(mjModel* m, mjData* d) override {
    m_ = m;
    d_ = d;
  }

  void SetExecutor(aimrt::executor::ExecutorRef executor) override {
    executor_ = executor;
  };

  void SetFreq(uint32_t freq) override {
    channel_frq_ = freq;
  };

  void PublishSensorData() override;

 private:
  void RegisterSensorAddr();

 private:
  struct SensorAddrGroup {
    int32_t jointpos_addr = -1;
    int32_t jointvel_addr = -1;
    int32_t jointactuatorfrc_addr = -1;
  };

  struct SensorStateGroup {
    double jointpos_state = 0.0;
    double jointvel_state = 0.0;
    double jointactuatorfrc_state = 0.0;
  };

 private:
  Options options_;

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;

  aimrt::channel::PublisherRef publisher_;
  aimrt::executor::ExecutorRef executor_;

  uint32_t channel_frq_ = 1000;
  double avg_interval_base_ = 1.0;
  double avg_interval_ = 0;

  uint32_t joint_num_ = 0;
  uint32_t counter_ = 0;

  std::vector<SensorAddrGroup> sensor_addr_vec_;
  std::vector<std::string> name_vec_;
};

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
