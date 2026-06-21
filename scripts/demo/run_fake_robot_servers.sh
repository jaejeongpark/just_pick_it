#!/usr/bin/env bash
# Fake robot servers를 Fleet/로봇과 같은 ROS discovery 환경에서 실행한다.
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"
if [[ "${USE_DDS:-1}" != "0" && -f "$ROOT_DIR/scripts/dds_env.sh" ]]; then
    source "$ROOT_DIR/scripts/dds_env.sh"
fi

set -u

echo "=== Fake Robot Servers (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}, ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-unset}) ==="
exec python3 "$ROOT_DIR/scripts/demo/fake_robot_servers.py" "$@"
