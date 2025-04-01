# Copyright (c) 2023, AgiBot Inc.
# All rights reserved.

set_namespace()

if(AIMRT_MUJOCO_SIM_BUILD_EXAMPLES)
  add_subdirectory(examples/inverted_pendulum)
  add_subdirectory(actuator)
endif()
