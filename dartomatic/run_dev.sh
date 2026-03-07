#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/build"
SOCK="/tmp/dart_appbus.sock"

mkdir -p "$ROOT/logs"

PIDS=()

cleanup() {
    echo "[run] stop..."
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}

trap cleanup INT TERM EXIT

echo "[run] démarrage..."

# AppBus
"$BIN/appbusd" > "$ROOT/logs/appbusd.log" 2>&1 &
PIDS+=($!)
sleep 0.2

# Services applicatifs
"$BIN/scoring_service" "$SOCK" > "$ROOT/logs/scoring.log" 2>&1 &
PIDS+=($!)

"$BIN/game_service" "$SOCK" > "$ROOT/logs/game.log" 2>&1 &
PIDS+=($!)

# Caméras
for cam_id in 0 2 4 6; do
    "$BIN/cam_process" "$cam_id" > "$ROOT/logs/cam_${cam_id}.log" 2>&1 &
    PIDS+=($!)
    sleep 0.2
done

# RT
"$BIN/rt_main" > "$ROOT/logs/rt.log" 2>&1 &
PIDS+=($!)

echo "[run] PIDs: ${PIDS[*]}"
echo "[run] logs dans $ROOT/logs/"
echo "[run] Ctrl+C pour arrêter."

wait