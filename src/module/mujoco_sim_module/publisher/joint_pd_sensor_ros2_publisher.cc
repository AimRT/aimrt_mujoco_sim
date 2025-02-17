// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/joint_pd_sensor_ros2_publisher.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher ::JointPdSensorRos2Publisher::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher ::JointPdSensorRos2Publisher::Options;

  static Node encode(const Options& rhs) {
    Node node;

    node["joints"] = YAML::Node();
    for (const auto& joint : rhs.joints) {
      Node joint_node;
      joint_node["name"] = joint.name;
      joint_node["bind_joint"] = joint.bind_joint;
      joint_node["bind_jointpos_sensor"] = joint.bind_jointpos_sensor;
      joint_node["bind_jointvel_sensor"] = joint.bind_jointvel_sensor;
      joint_node["bind_jointactuatorfrc_sensor"] = joint.bind_jointactuatorfrc_sensor;
      node["joints"].push_back(joint_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["joints"] && node["joints"].IsSequence()) {
      for (const auto& joint_node : node["joints"]) {
        auto joint_node_options = Options::Joint();
        joint_node_options.name = joint_node["name"].as<std::string>();
        joint_node_options.bind_joint = joint_node["bind_joint"].as<std::string>();

        if (joint_node["bind_jointpos_sensor"]) {
          joint_node_options.bind_jointpos_sensor = joint_node["bind_jointpos_sensor"].as<std::string>();
        }
        if (joint_node["bind_jointvel_sensor"]) {
          joint_node_options.bind_jointvel_sensor = joint_node["bind_jointvel_sensor"].as<std::string>();
        }
        if (joint_node["bind_jointactuatorfrc_sensor"]) {
          joint_node_options.bind_jointactuatorfrc_sensor = joint_node["bind_jointactuatorfrc_sensor"].as<std::string>();
        }

        rhs.joints.emplace_back(std::move(joint_node_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::publisher {

void JointPdSensorRos2Publisher::Initialize(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);

  RegisterSensorAddr();

  options_node = options_;

  bool ret = aimrt::channel::RegisterPublishType<sensor_ros2::msg::JointPdState>(publisher_);

  AIMRT_CHECK_ERROR_THROW(ret, "Register publish type failed.");
}

void JointPdSensorRos2Publisher::PublishSensorData() {
  static constexpr uint32_t ONE_MB = 1024 * 1024;

  if (counter_++ < avg_interval_) return;

  std::unique_ptr<SensorStateGroup[]> state_array(new SensorStateGroup[joint_num_]);

  // if not define specific sensor , its value is set to 0.0
  for (size_t i = 0; i < joint_num_; i++) {
    const auto& addr = sensor_addr_vec_[i];
    state_array[i].jointpos_state = addr.jointpos_addr >= 0 ? d_->sensordata[addr.jointpos_addr] : 0.0;
    state_array[i].jointvel_state = addr.jointvel_addr >= 0 ? d_->sensordata[addr.jointvel_addr] : 0.0;
    state_array[i].jointactuatorfrc_state = addr.jointactuatorfrc_addr >= 0 ? d_->sensordata[addr.jointactuatorfrc_addr] : 0.0;
  }

  executor_.Execute([this, state_array = std::move(state_array)]() {
    sensor_ros2::msg::JointPdState state;

    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    state.header.stamp.sec = timestamp / 1e9;
    state.header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
    state.header.frame_id = "touch_sensor";

    state.joints.resize(joint_num_);
    for (int i = 0; i < joint_num_; ++i) {
      sensor_ros2::msg::State joint_state;
      joint_state.name = name_vec_[i];
      joint_state.position = state_array[i].jointpos_state;
      joint_state.velocity = state_array[i].jointvel_state;
      joint_state.effort = state_array[i].jointactuatorfrc_state;
      state.joints[i] = joint_state;
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

void JointPdSensorRos2Publisher::RegisterSensorAddr() {
  for (const auto& joint : options_.joints) {
    sensor_addr_vec_.emplace_back(SensorAddrGroup{
        .jointpos_addr = GetSensorAddr(m_, joint.bind_jointpos_sensor),
        .jointvel_addr = GetSensorAddr(m_, joint.bind_jointvel_sensor),
        .jointactuatorfrc_addr = GetSensorAddr(m_, joint.bind_jointactuatorfrc_sensor)});

    name_vec_.emplace_back(joint.name);
  }

  joint_num_ = sensor_addr_vec_.size();
}

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher
