#!/bin/bash

set -e

source install/share/sensor_ros2/local_setup.bash

./aimrt_main --cfg_file_path=./cfg/examples_hardware_touch_sensor_ros2_cfg.yaml
