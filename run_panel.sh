#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec python -u -m x2_kinematics.aimdk_bridge.dashboard "$@"
