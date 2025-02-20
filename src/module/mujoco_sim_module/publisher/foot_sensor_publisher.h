// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#pragma once

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "foot_sensor_state.pb.h"
#include "mujoco_sim_module/global.h"
#include "mujoco_sim_module/publisher/publisher_base.h"
#include "mujoco_sim_module/publisher/utils.h"

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {
class FootSensorPublisher : public PublisherBase {
 public:
  struct Options {
    std::vector<std::string> names;

    struct State {
      std::string bind_site;
      std::string bind_foot_sensor;
    };
    std::vector<std::vector<State>> states;
  };

 public:
  FootSensorPublisher() = default;
  ~FootSensorPublisher() override = default;

  void Initialize(YAML::Node options_node) override;
  void Start() override {}
  void Shutdown() override {}

  [[nodiscard]] std::string_view Type() const noexcept override { return "foot_sensor"; }

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
    std::vector<int32_t> addr_vec;
  };

  struct SensorStateGroup {
    std::vector<double> state_vec;
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

  uint32_t foot_sensor_group_num_ = 0;
  uint32_t counter_ = 0;

  std::vector<SensorAddrGroup> sensor_addr_group_vec_;
  std::vector<std::string> name_vec_;
  std::vector<uint32_t> foot_sensor_num_vec_;
};
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher