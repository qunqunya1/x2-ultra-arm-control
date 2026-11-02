#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/env.sh"
set -u
CONDA="${X2_CONDA_EXE:-/home/dev/miniconda3/bin/conda}"
[[ -x $CONDA ]] || { echo 'Install the dedicated Miniconda environment first.' >&2; exit 1; }
cd -- "$HERE/../.."
exec "$CONDA" run --no-capture-output -n "${X2_CONDA_ENV:-x2-control}" \
    python -B -u -m x2_kinematics.aimdk_bridge.dashboard "$@"
