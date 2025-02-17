#!/bin/bash

# default values
DEFAULT_LEFT_MOTOR=0.5
DEFAULT_RIGHT_MOTOR=0.3

# read command line arguments
RIGHT_MOTOR=${2:-$DEFAULT_RIGHT_MOTOR}
LEFT_MOTOR=${1:-$DEFAULT_LEFT_MOTOR}

# build json data
JSON_DATA=$(cat <<EOF
{
  "data": [
    {
      "name": "right_motor",
      "position": 0.0,
      "velocity": 0.0,
      "effort": $RIGHT_MOTOR,
      "stiffness": 1.0,
      "damping": 1.0
    },
    {
      "name": "left_motor",
      "position": 0.0,
      "velocity": 0.0,
      "effort": $LEFT_MOTOR,
      "stiffness": 1.0,
      "damping": 1.0
    }
  ]
}
EOF
)

# send json data to server
curl -i \
  -H 'content-type:application/json' \
  -X POST 'http://127.0.0.1:50080/channel/%2Fexamples_hardware%2Fjoint%2Fjoint_pd_command/pb%3Aaimrt.protocols.sensor.JointPdState' \
  -d "$JSON_DATA"