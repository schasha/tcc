#!/usr/bin/env bash
# run_all.sh — runs the experiments of bench/README.md and stores the raw
# results under $OUT (default bench/results/<host>-<date>).
#
# Run inside the ROS environment with the package built and sourced
# (docker compose run --rm -T ros bash -lc 'bench/run_all.sh'), or on any
# Linux host with ROS 2 Humble and the workspace installed.
#
# Environment:
#   REPS=30         repetitions of each isolated configuration (H1, H3)
#   REPS_OL=5       repetitions of each open-loop rate (H2)
#   REPS_E2E=5      repetitions of each end-to-end rate (H4)
#   CPU=            pin the measuring thread to this CPU (e.g. 2); empty = no pin
#   ONLY=           space-separated subset of: env h1 conv h3 h2 h4 h5
#   GAZEBO=0        1 also runs H5 (Gazebo, several minutes per repetition)
#   QUICK=0         1 = few repetitions and short runs, to check the pipeline
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${BIN:-$(ros2 pkg prefix semantic_gateway)/lib/semantic_gateway}"
OUT="${OUT:-$HERE/results/$(hostname)-$(date +%Y%m%d-%H%M%S)}"
REPS="${REPS:-30}"; REPS_OL="${REPS_OL:-5}"; REPS_E2E="${REPS_E2E:-5}"
ONLY="${ONLY:-env h1 conv h3 h2 h4}"; [ "${GAZEBO:-0}" = 1 ] && ONLY="$ONLY h5"
ITERS=20000; ITERS_SEND=2000; OL_SECS=2; E2E_SECS=10; CONV_ITERS=200000
if [ "${QUICK:-0}" = 1 ]; then
    REPS=3; REPS_OL=1; REPS_E2E=1; ITERS=2000; ITERS_SEND=200; OL_SECS=0.5; E2E_SECS=3; CONV_ITERS=20000
fi
PIN=(); [ -n "${CPU:-}" ] && PIN=(--cpu "$CPU")
mkdir -p "$OUT/raw"
want() { [[ " $ONLY " == *" $1 "* ]]; }
log() { echo "[$(date +%T)] $*"; }

# ── environment: what the numbers depend on ─────────────────────────────────
if want env; then
    {
        echo "date: $(date -Is)"
        echo "host: $(hostname)"
        echo "kernel: $(uname -srvm)"
        grep -m1 "model name" /proc/cpuinfo || true
        echo "cpus: $(nproc)"
        echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo n/a)"
        grep -qi microsoft /proc/version && echo "virtualization: WSL2" \
            || echo "virtualization: $(systemd-detect-virt 2>/dev/null || echo unknown)"
        echo "preempt: $(uname -v | grep -o 'PREEMPT[_A-Z]*' || echo none)"
        echo "rmw: ${RMW_IMPLEMENTATION:-default}"
        echo "pin: ${CPU:-none}"
        echo "git: $(git -C "$HERE/.." rev-parse --short HEAD 2>/dev/null || echo n/a)"
    } > "$OUT/env.txt"
    cat "$OUT/env.txt"
fi

# ── H1: translation latency vs. message size ────────────────────────────────
if want h1; then
    log "H1: translation latency vs. size ($REPS reps)"
    for r in $(seq 1 "$REPS"); do
        for b in 10 100 360 1000 4000; do
            raw=(); [ "$r" = 1 ] && raw=(--raw "$OUT/raw/h1_scan_$b.txt")
            "$BIN/bench_gateway" translate --msg scan --beams "$b" --iters "$ITERS" --tag "$r" \
                --out "$OUT/h1.csv" "${PIN[@]}" "${raw[@]}"
        done
        for m in imu temperature; do
            raw=(); [ "$r" = 1 ] && raw=(--raw "$OUT/raw/h1_$m.txt")
            "$BIN/bench_gateway" translate --msg "$m" --iters "$ITERS" --tag "$r" \
                --out "$OUT/h1.csv" "${PIN[@]}" "${raw[@]}"
        done
    done
fi

# ── convergence: one long series, percentiles over growing prefixes ─────────
if want conv; then
    log "convergence: $CONV_ITERS iterations"
    "$BIN/bench_gateway" translate --msg scan --beams 100 --iters "$CONV_ITERS" --tag conv \
        --out "$OUT/conv.csv" --raw "$OUT/raw/conv_scan_100.txt" "${PIN[@]}"
fi

# ── H3: cost of sending, gateway vs. bare sendto ────────────────────────────
if want h3; then
    log "H3: send cost ($REPS reps)"
    for r in $(seq 1 "$REPS"); do
        for k in 0 1 10 50 100; do
            "$BIN/bench_gateway" translate --msg scan --beams 100 --k "$k" --iters "$ITERS_SEND" \
                --warmup 200 --tag "$r" --out "$OUT/h3_gateway.csv" "${PIN[@]}"
        done
        for k in 1 10 50 100; do
            "$BIN/bench_gateway" sendto --via raw     --k "$k" --iters "$ITERS_SEND" --tag "$r" --out "$OUT/h3_sendto.csv" "${PIN[@]}"
            "$BIN/bench_gateway" sendto --via raw --listen 1 --k "$k" --iters "$ITERS_SEND" --tag "$r" --out "$OUT/h3_sendto.csv" "${PIN[@]}"
            "$BIN/bench_gateway" sendto --via network --k "$k" --iters "$ITERS_SEND" --tag "$r" --out "$OUT/h3_sendto.csv" "${PIN[@]}"
        done
    done
fi

# ── H2: open-loop load until saturation ─────────────────────────────────────
if want h2; then
    log "H2: open loop ($REPS_OL reps)"
    for r in $(seq 1 "$REPS_OL"); do
        for rate in 1000 10000 30000 60000 90000 120000 140000 160000 200000; do
            "$BIN/bench_gateway" openloop --msg scan --beams 100 --k 0 --rate "$rate" --duration "$OL_SECS" \
                --tag "$r" --out "$OUT/h2.csv" "${PIN[@]}"
        done
        for rate in 50 100 200 400 600 800 1000 1500 2000; do
            "$BIN/bench_gateway" openloop --msg scan --beams 100 --k 100 --rate "$rate" --duration "$OL_SECS" \
                --tag "$r" --out "$OUT/h2.csv" "${PIN[@]}"
        done
    done
fi

# ── H4: end to end over ROS 2 + SmartData network ───────────────────────────
if want h4; then
    log "H4: end to end ($REPS_E2E reps)"
    export ROS_DOMAIN_ID="${E2E_DOMAIN:-77}"
    port=$((45000 + RANDOM % 4000))
    for r in $(seq 1 "$REPS_E2E"); do
        for rate in 100 500 1000 2000 5000; do
            pids=()
            /opt/ros/humble/lib/tf2_ros/static_transform_publisher --frame-id map --child-frame-id imu_frame \
                >/dev/null 2>&1 & pids+=($!)
            "$BIN/gateway_node" --ros-args -p enable_ros_mirror:=false -p bus_port:=$port \
                -p stats_interval_s:=3600 >/dev/null 2>&1 & pids+=($!)
            "$BIN/proxy_node" --ros-args -p bus_port:=$port >/dev/null 2>&1 & pids+=($!)
            sleep 3  # discovery + interests bound
            expected=$(python3 -c "print(int($rate*$E2E_SECS))")
            "$BIN/bench_e2e" probe --ros-args -p out:="$OUT/h4.csv" -p tag:="r$r" -p rate:="$rate.0" \
                -p expected:="$expected" -p listen:="$(python3 -c "print(float($E2E_SECS+3))")" \
                -p raw_prefix:="$OUT/raw/h4_${rate}_$r" >/dev/null 2>&1 & probe=$!
            sleep 1
            "$BIN/bench_e2e" pub --ros-args -p rate:="$rate.0" -p duration:="$(python3 -c "print(float($E2E_SECS))")" >/dev/null 2>&1
            wait "$probe"
            kill "${pids[@]}" 2>/dev/null || true
            wait "${pids[@]}" 2>/dev/null || true
            log "  H4 rep $r rate $rate done"
        done
    done
fi

# ── H5: setpoint steps through SmartData in Gazebo ──────────────────────────
if want h5; then
    log "H5: Gazebo setpoint steps"
    export ROS_DOMAIN_ID="${E2E_DOMAIN:-78}"
    for r in $(seq 1 "${REPS_GZ:-3}"); do
        ros2 launch semantic_gateway gazebo_gateway.launch.py gui:=false drive:=false \
            world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/empty_world.world \
            >"$OUT/raw/h5_launch_$r.log" 2>&1 & launch=$!
        # empty world: nothing to collide with while turning. bench_steps waits
        # for /odom itself, i.e. for the robot to be spawned
        timeout "${GZ_TIMEOUT:-600}" "$BIN/bench_steps" --ros-args -p use_sim_time:=true \
            -p out:="$OUT/raw/h5_$r" >"$OUT/raw/h5_steps_$r.log" 2>&1 || log "  bench_steps failed/timed out"
        # background jobs ignore SIGINT in a script: terminate the launch tree
        kill -TERM "$launch" 2>/dev/null || true
        sleep 5
        pkill -f gzserver 2>/dev/null || true
        pkill -f gzclient 2>/dev/null || true
        pkill -f "semantic_gateway/lib/semantic_gateway/(gateway|proxy)_node" 2>/dev/null || true
        pkill -f robot_state_publisher 2>/dev/null || true
        wait "$launch" 2>/dev/null || true
        sleep 3
        log "  H5 rep $r done"
    done
fi

log "results in $OUT — run: python3 $HERE/analyze.py $OUT"
