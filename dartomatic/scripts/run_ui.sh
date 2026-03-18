#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/build"
SOCK="/tmp/dart_appbus.sock"

LOGDIR="$ROOT/logs"
mkdir -p "$LOGDIR"

PIDS=()
CAM_IDS=(0 2 4 6)

cleanup() {
    trap - INT TERM HUP EXIT
    echo "[run_ui] stop..."

    # Tuer tout le groupe de processus lancé par ce script
    kill -- -$$ 2>/dev/null || true

    wait 2>/dev/null || true
}

trap cleanup INT TERM HUP EXIT

echo "[run_ui] démarrage..."

# -------------------------
# AppBus
# -------------------------
"$BIN/appbusd" > "$LOGDIR/appbusd.log" 2>&1 &
PIDS+=($!)
sleep 0.3

# -------------------------
# Services
# -------------------------
"$BIN/scoring_service" "$SOCK" > "$LOGDIR/scoring.log" 2>&1 &
PIDS+=($!)

"$BIN/game_service" "$SOCK" > "$LOGDIR/game.log" 2>&1 &
PIDS+=($!)

# -------------------------
# Cameras
# -------------------------
echo "[run_ui] lancement cam_process..."

for cam_id in "${CAM_IDS[@]}"; do
    "$BIN/cam_process" "$cam_id" > "$LOGDIR/cam_${cam_id}.log" 2>&1 &
    PIDS+=($!)
    sleep 0.5
done

# -------------------------
# RT
# -------------------------
"$BIN/rt_main" > "$LOGDIR/rt.log" 2>&1 &
PIDS+=($!)

# -------------------------
# Attente cams prêtes
# -------------------------
echo "[run_ui] attente caméras prêtes..."

READY_COUNT=0

for i in {1..10}; do
    READY_COUNT=0

    for cam_id in "${CAM_IDS[@]}"; do
        if grep -q "Reference ready, waiting RtTriggerCmd" \
            "$LOGDIR/cam_${cam_id}.log" 2>/dev/null; then
            READY_COUNT=$((READY_COUNT + 1))
        fi
    done

    echo "[run_ui] cams prêtes: $READY_COUNT"

    if [ "$READY_COUNT" -ge 2 ]; then
        break
    fi

    sleep 1
done

if [ "$READY_COUNT" -lt 2 ]; then
    echo "[run_ui] ERREUR: moins de 2 caméras prêtes."
    echo "[run_ui] voir logs dans $LOGDIR"
    exit 1
fi

echo "[run_ui] OK -> au moins 2 caméras prêtes"

# -------------------------
# UI au premier plan
# -------------------------
echo "[run_ui] lancement UI"
"$BIN/main_menu" "$SOCK"