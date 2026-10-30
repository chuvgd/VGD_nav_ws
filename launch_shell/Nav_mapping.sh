#!/bin/bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/nav_env.sh"

exec ros2 launch pb2025_sentry_bringup test.launch.py \
    world:=VGD \
    use_rviz:=True \
    use_robot_state_pub:=True \
    slam:=True \
    "$@"
