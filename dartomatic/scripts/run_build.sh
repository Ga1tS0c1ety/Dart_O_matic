#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

rm -rf build
cmake -S . -B build
cmake --build build -j

./scripts/run_ui.sh