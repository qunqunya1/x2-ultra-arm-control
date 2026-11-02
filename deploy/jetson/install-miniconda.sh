#!/usr/bin/env bash
# Run only inside the project's container. Persistent writes stay in its HOME.
set -euo pipefail
[[ ${HOME:-} == /home/dev && -d /workspace/sdk/src ]] || {
    echo 'Run this script inside the project container with HOME=/home/dev.' >&2
    exit 1
}
[[ $(uname -m) == aarch64 ]] || { echo 'This installer requires Linux ARM64.' >&2; exit 1; }
prefix=/home/dev/miniconda3
filename=Miniconda3-py310_26.9.4-0-Linux-aarch64.sh
expected=fcbe104a99e0dcaee79748bf691b24bf6793daed397ac8eb49c064d57a394c1c
url="https://repo.anaconda.com/miniconda/$filename"
mkdir -p /home/dev/installers /workspace/logs
installer="/home/dev/installers/$filename"
if [[ ! -x "$prefix/bin/conda" ]]; then
    [[ ! -e "$prefix" ]] || { echo 'Partial or unknown installation exists; refusing to overwrite it.' >&2; exit 1; }
    if [[ ! -f "$installer" ]]; then
        curl -fL --connect-timeout 15 --max-time 1800 --retry 3 --retry-delay 3 \
            -o "$installer.partial" "$url"
        printf '%s  %s\n' "$expected" "$installer.partial" | sha256sum -c -
        mv "$installer.partial" "$installer"
    fi
    printf '%s  %s\n' "$expected" "$installer" | sha256sum -c -
    env -u PYTHONPATH -u PYTHONHOME bash "$installer" -b -p "$prefix"
fi
"$prefix/bin/conda" config --file /home/dev/.condarc --set auto_activate_base false
touch /home/dev/.bashrc
if ! grep -Fq '# >>> xlq miniconda >>>' /home/dev/.bashrc; then
    cat >> /home/dev/.bashrc <<'BASHRC'

# >>> xlq miniconda >>>
# Make conda available without activating base or changing the ROS Python.
if [ -r /home/dev/miniconda3/etc/profile.d/conda.sh ]; then
    . /home/dev/miniconda3/etc/profile.d/conda.sh
fi
# <<< xlq miniconda <<<
BASHRC
fi
"$prefix/bin/conda" --version
"$prefix/bin/python" --version
"$prefix/bin/conda" info --json > /workspace/logs/miniconda-info.json
printf 'source_url=%s\nsha256=%s\nprefix=%s\n' "$url" "$expected" "$prefix" \
    > /workspace/logs/miniconda-install-source.txt
echo MINICONDA_INSTALL_OK
