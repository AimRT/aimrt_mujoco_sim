// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/touch_sensor_publisher.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher ::TouchSensorPublisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher ::TouchSensorPublisher::Options;

  static YAML::Node encode(const Options& rhs) {
    YAML::Node node;

    YAML::Node names_node = YAML::Node();
    for (const auto& name : rhs.names) {
      names_node.push_back(name);
    }
    node["names"] = names_node;

    YAML::Node states_node = YAML::Node();
    for (const auto& state_group : rhs.states) {
      YAML::Node inner_states_node = YAML::Node();
      for (const auto& state : state_group) {
        YAML::Node state_node;
        state_node["bind_site"] = state.bind_site;
        state_node["bind_touch_sensor"] = state.bind_touch_sensor;
        inner_states_node.push_back(state_node);
      }
      states_node.push_back(inner_states_node);
    }
    node["states"] = states_node;

    return node;
  }

  static bool decode(const YAML::Node& node, Options& rhs) {
    if (node["names"] && node["names"].IsSequence()) {
      for (const auto& name_node : node["names"]) {
        rhs.names.push_back(name_node.as<std::string>());
      }
    }

    if (node["states"] && node["states"].IsSequence()) {
      for (const auto& state_group_node : node["states"]) {
        if (state_group_node.IsSequence()) {
          std::vector<Options::State> state_group;

          for (const auto& state_node : state_group_node) {
            Options::State state_options;

            if (state_node["bind_site"]) {
              state_options.bind_site = state_node["bind_site"].as<std::string>();
            }
            if (state_node["bind_touch_sensor"]) {
              state_options.bind_touch_sensor = state_node["bind_touch_sensor"].as<std::string>();
            }
            state_group.emplace_back(std::move(state_options));
          }
          rhs.states.emplace_back(std::move(state_group));
        }
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

void TouchSensorPublisher::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);

  RegisterSensorAddr();

  options_node = options_;

  bool ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::TouchState>(publisher_);

  AIMRT_CHECK_ERROR_THROW(ret, "Register touch sensor publish type failed.");
}

void TouchSensorPublisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  std::unique_ptr<SensorStateGroup[]> state_array(new SensorStateGroup[touch_group_num_]);

  // if not define specific sensor , its value is set to 0.0
  for (size_t i = 0; i < touch_group_num_; i++) {
    const auto& addr_vec = sensor_addr_group_vec_[i].addr_vec;
    state_array[i].state_vec.reserve(touch_num_vec_[i]);
    std::transform(addr_vec.begin(),
                   addr_vec.end(),
                   state_array[i].state_vec.begin(),
                   [this](int addr) { return addr < 0 ? 0.0 : d_->sensordata[addr]; });
  }

  executor_.Execute([this, state_array = std::move(state_array)]() {
    aimrt::protocols::sensor::TouchState state;
    state.mutable_names()->Reserve(touch_group_num_);
    state.mutable_states()->Reserve(touch_group_num_);

    for (int i = 0; i < touch_group_num_; ++i) {
      state.add_names(name_vec_[i]);
      auto* states = state.add_states();
      states->mutable_pressure()->Reserve(touch_num_vec_[i]);
      for (int j = 0; j < touch_num_vec_[i]; ++j) {
        states->add_pressure(state_array[i].state_vec[j]);
      }
    }

    aimrt::channel::Publish(publisher_, state);
  });

  avg_interval_ += avg_interval_base_;

  // avoid overflow
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

void TouchSensorPublisher::RegisterSensorAddr() {
  touch_group_num_ = options_.names.size();

  for (size_t index = 0; index < touch_group_num_; ++index) {
    name_vec_.emplace_back(options_.names[index]);

    std::vector<int32_t> addr_vec;
    std::transform(options_.states[index].begin(),
                   options_.states[index].end(),
                   std::back_inserter(addr_vec),
                   [this](const Options::State& state) {
                     return GetSensorAddr(m_, state.bind_touch_sensor);
                   });

    touch_num_vec_.emplace_back(addr_vec.size());

    sensor_addr_group_vec_.emplace_back(SensorAddrGroup{
        .addr_vec = std::move(addr_vec)});
  }
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
