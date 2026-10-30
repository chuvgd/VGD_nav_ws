#!/bin/bash
# Shared by the real-robot launch scripts. NAV_WS optionally selects another workspace.
set -eo pipefail

nav_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
nav_workspace="$(cd -- "${NAV_WS:-${nav_script_dir}/..}" && pwd)"
nav_ros_setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
for nav_setup in "$nav_ros_setup" "$nav_workspace/install/setup.bash"; do
    if [[ ! -f "$nav_setup" ]]; then
        printf 'Missing ROS setup: %s\n' "$nav_setup" >&2
        exit 1
    fi
done

# MVS ships a libusb that conflicts with ROS/PCL. Keep other library paths intact.
unset MVCAM_SDK_PATH MVCAM_COMMON_RUNENV MVCAM_GENICAM_CLPROTOCOL ALLUSERSPROFILE
nav_library_paths=""
IFS=: read -r -a nav_library_entries <<< "${LD_LIBRARY_PATH:-}"
for nav_library in "${nav_library_entries[@]}"; do
    case "$nav_library" in
        /opt/MVS/*|"") ;;
        *) nav_library_paths="${nav_library_paths:+${nav_library_paths}:}${nav_library}" ;;
    esac
done
export LD_LIBRARY_PATH="$nav_library_paths"
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libusb-1.0.so"

source "$nav_ros_setup"
source "$nav_workspace/install/setup.bash"

# Fail instead of silently launching another overlay's copy of this package.
nav_package_prefix="$(ros2 pkg prefix pb2025_sentry_bringup)"
case "$nav_package_prefix/" in
    "$nav_workspace/install/"*) ;;
    *)
        printf 'Wrong bringup overlay: %s (expected %s/install)\n' \
            "$nav_package_prefix" "$nav_workspace" >&2
        exit 1
        ;;
esac
printf 'Navigation workspace: %s\nBringup package: %s\nDISPLAY: %s\n' \
    "$nav_workspace" "$nav_package_prefix" "${DISPLAY:-<unset>}"
