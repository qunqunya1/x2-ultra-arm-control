#!/usr/bin/env bash
# Source in the container; all runtime mounts and SDK inputs are project scoped.
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo 'Missing mounted ROS Humble setup.' >&2
    return 1
fi
_x2_restore_nounset=false
[[ $- != *u* ]] || _x2_restore_nounset=true
set +u
source /opt/ros/humble/setup.bash
if [[ -f /workspace/sdk/install/local_setup.bash ]]; then
    source /workspace/sdk/install/local_setup.bash
fi
if [[ $_x2_restore_nounset == true ]]; then set -u; fi
unset _x2_restore_nounset
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${X2_DDS_PROFILE:-/workspace/config/dds-panel.xml}"
if [[ ! -r $FASTRTPS_DEFAULT_PROFILES_FILE ]]; then
    echo 'Generate the project DDS profile with make_dds_profile.py first.' >&2
    return 1
fi
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
