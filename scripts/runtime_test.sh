#!/usr/bin/env bash
# Run the installed smoke test in a fresh, isolated Gazebo Fortress world.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

gui=${1:-false}
if [[ "$gui" != true && "$gui" != false ]]; then
    echo 'Usage: bash scripts/runtime_test.sh [true|false] (show GUI)' >&2
    exit 2
fi
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=${BENI_TEST_DOMAIN_ID:-68}
export IGN_PARTITION="beni_test_$$"
result_dir="runtime_results/$(date -u +%Y%m%dT%H%M%SZ)_$$"
mkdir -p "$result_dir"
export ROS_LOG_DIR="$PWD/$result_dir/ros"
echo "Runtime evidence: $PWD/$result_dir"
{
    date -u --iso-8601=seconds
    cat /etc/os-release
    ign gazebo --versions
    dpkg-query -W -f='${Package} ${Version}\n' \
        ros-humble-rclpy ros-humble-ros-gz-bridge libignition-gazebo6
    printf 'gui=%s ROS_DOMAIN_ID=%s IGN_PARTITION=%s\n' \
        "$gui" "$ROS_DOMAIN_ID" "$IGN_PARTITION"
} > "$result_dir/environment.log" 2>&1

setsid ros2 launch beni_sim sim.launch.py "gui:=$gui" \
    > "$result_dir/gazebo.log" 2>&1 &
launch_pid=$!
cleanup() {
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for ((i=0; i<100; i++)); do
        if ! kill -0 -- "-$launch_pid" 2>/dev/null; then break; fi
        sleep 0.1
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sleep 3
if ! kill -0 "$launch_pid" 2>/dev/null; then
    cat "$result_dir/gazebo.log" >&2
    exit 1
fi
set +e
timeout --signal=INT --kill-after=5s 75s ros2 run beni_sim smoke_test \
    2>&1 | tee "$result_dir/smoke_test.log"
result=${PIPESTATUS[0]}
set -e
printf '%s\n' "$result" > "$result_dir/exit_code.txt"
exit "$result"
