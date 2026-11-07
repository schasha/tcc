#!/usr/bin/env bash
# notebook.sh — runs the whole benchmark on a native Linux host using only
# this repository and Docker (no ROS installation needed).
#
#   bench/notebook.sh            # everything, including Gazebo (H5)
#   QUICK=1 bench/notebook.sh    # a few minutes, just to check it works
#
# Results end up in bench/results/<name>/ (summary.md, CSVs, plots/).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

NAME="${NAME:-$(hostname)-$(date +%Y%m%d-%H%M)}"
command -v docker >/dev/null || { echo "Docker não encontrado: sudo apt install docker.io docker-compose-v2"; exit 1; }

# best effort: a fixed CPU frequency reduces noise (needs sudo; skipped if unavailable)
if command -v cpupower >/dev/null && sudo -n true 2>/dev/null; then
    sudo cpupower frequency-set -g performance >/dev/null || true
fi

export USER_UID="$(id -u)" USER_GID="$(id -g)"
docker compose build ros
docker compose run --rm -T ros bash -lc "
    set -e
    cd /ros2_ws
    colcon build --packages-select semantic_gateway --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source install/setup.bash
    GAZEBO=${GAZEBO:-1} QUICK=${QUICK:-0} OUT=/ros2_ws/src/semantic_gateway/bench/results/$NAME \
        src/semantic_gateway/bench/run_all.sh
    python3 src/semantic_gateway/bench/analyze.py src/semantic_gateway/bench/results/$NAME >/dev/null
"
echo
echo "Pronto: bench/results/$NAME/summary.md"
