#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/navigation/sync_to_picky2.sh <robot-ip-or-user@host> [remote_repo_dir]
  bash scripts/navigation/sync_to_picky2.sh --dry-run <robot-ip-or-user@host> [remote_repo_dir]

Examples:
  bash scripts/navigation/sync_to_picky2.sh 192.168.1.42
  bash scripts/navigation/sync_to_picky2.sh pinky@192.168.1.42
  bash scripts/navigation/sync_to_picky2.sh --dry-run pinky@192.168.1.42

Default remote_repo_dir:
  /home/pinky/just_pick_it

This syncs only:
  src/just_pick_it/pinky_amr_2
EOF
}

dry_run=0
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${1:-}" == "-n" || "${1:-}" == "--dry-run" ]]; then
  dry_run=1
  shift
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage >&2
  exit 2
fi

if ! command -v rsync >/dev/null 2>&1; then
  echo "[sync] rsync command not found. Install rsync first." >&2
  exit 1
fi

robot_host="$1"
remote_repo_dir="${2:-/home/pinky/just_pick_it}"

if [[ "$robot_host" != *@* ]]; then
  robot_host="pinky@${robot_host}"
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
package_rel="src/just_pick_it/pinky_amr_2"
source_dir="${repo_root}/${package_rel}"
target_dir="${remote_repo_dir%/}/${package_rel}"

rsync_args=(
  -av
  --delete
  --human-readable
  --exclude .pytest_cache
  --exclude .mypy_cache
  --exclude .ruff_cache
  --exclude __pycache__
  --exclude '*.pyc'
  --exclude '*.db'
  --exclude '*.sqlite'
  --exclude '*.sqlite3'
  --exclude '*.env'
)

if [[ "$dry_run" -eq 1 ]]; then
  rsync_args+=(--dry-run)
  echo "[sync] dry run only. No files will be copied."
fi

echo "[sync] source: ${source_dir}/"
echo "[sync] target: ${robot_host}:${target_dir}/"

ssh "$robot_host" "mkdir -p '${target_dir}'"
rsync "${rsync_args[@]}" "${source_dir}/" "${robot_host}:${target_dir}/"

echo "[sync] done."
echo "[sync] on robot, restart Nav2/State Machine or run:"
echo "       cd ${remote_repo_dir%/} && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select pinky_amr_2"
