#!/bin/bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/nav_env.sh"

exec ros2 launch waypoint_editor waypoint_editor.launch.py
