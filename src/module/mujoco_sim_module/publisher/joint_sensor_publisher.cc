// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/publisher/joint_sensor_publisher.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::publisher ::JointSensorPublisherBase::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::publisher ::JointSensorPublisherBase::Options;

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

void JointSensorPublisherBase ::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void JointSensorPublisherBase::RegisterSensorAddr() {
  for (const auto& joint : options_.joints) {
    sensor_addr_vec_.emplace_back(SensorAddrGroup{
        .jointpos_addr = GetSensorAddr(m_, joint.bind_jointpos_sensor),
        .jointvel_addr = GetSensorAddr(m_, joint.bind_jointvel_sensor),
        .jointactuatorfrc_addr = GetSensorAddr(m_, joint.bind_jointactuatorfrc_sensor)});

    name_vec_.emplace_back(joint.name);
  }

  joint_num_ = sensor_addr_vec_.size();
}

void JointSensorPublisherBase::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  avg_interval_base_ = GetAvgIntervalBase(channel_frq_);

  RegisterSensorAddr();

  options_node = options_;
}

void JointSensorPublisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::JointState>(publisher_),
                          "Register publish type failed.");
}

void JointSensorPublisher::PublishSensorData() {
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
    aimrt::protocols::sensor::JointState joint_state;
    for (int i = 0; i < joint_num_; ++i) {
      auto* data = joint_state.add_joints();
      data->set_name(name_vec_[i]);

      data->set_position(state_array[i].jointpos_state);
      data->set_velocity(state_array[i].jointvel_state);
      data->set_effort(state_array[i].jointactuatorfrc_state);
    }

    aimrt::channel::Publish(publisher_, joint_state);
  });

  avg_interval_ += avg_interval_base_;

  // avoid overflow
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
void JointSensorRos2Publisher::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::RegisterPublishType<sensor_ros2::msg::JointState>(publisher_),
                          "Register publish type failed.");
}

void JointSensorRos2Publisher::PublishSensorData() {
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
    sensor_ros2::msg::JointState joint_state;

    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    joint_state.header.stamp.sec = timestamp / 1e9;
    joint_state.header.stamp.nanosec = timestamp % static_cast<uint64_t>(1e9);
    joint_state.header.frame_id = "joint_sensor_ros2";

    joint_state.joints.resize(joint_num_);
    for (int i = 0; i < joint_num_; ++i) {
      sensor_ros2::msg::SingleJointState state;
      state.name = name_vec_[i];
      state.position = state_array[i].jointpos_state;
      state.velocity = state_array[i].jointvel_state;
      state.effort = state_array[i].jointactuatorfrc_state;
      joint_state.joints[i] = state;
    }

    aimrt::channel::Publish(publisher_, joint_state);
  });

  avg_interval_ += avg_interval_base_;

  // avoid overflow
  if (counter_ > ONE_MB) {
    avg_interval_ -= ONE_MB;
    counter_ -= ONE_MB;
  }
}
#endif

}  // namespace aimrt_mujoco_sim::mujoco_sim_module::publisher