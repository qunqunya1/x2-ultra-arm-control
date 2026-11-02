# Pull the base image via the NJU NGC mirror; nvcr.io is often unreachable in China.
FROM ngc.nju.edu.cn/nvidia/l4t-base:r36.2.0

LABEL agi.image.purpose="ROS Humble base, no CUDA bundled (mount /usr/local/cuda + /usr/lib/.../libcudnn,libnvinfer at runtime)"
LABEL agi.image.base="l4t-base:r36.2.0"
LABEL agi.image.note="Mount /opt/ros, CUDA toolkit, cuDNN/TensorRT shared libs from host at runtime"

ARG BUILD_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# Point the container's system apt at the Aliyun ubuntu-ports mirror: the base image
# defaults to ports.ubuntu.com, which is slow in China, and the hundreds of megabytes
# of build-essential / cmake / OpenCV dependencies below all come from this source.
# Also disable the NVIDIA L4T apt source; this image installs no l4t packages.
RUN for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do \
        [ -f "$f" ] || continue; \
        sed -i -E \
            -e 's@https?://ports\.ubuntu\.com/ubuntu-ports@https://mirrors.aliyun.com/ubuntu-ports@g' \
            -e 's@https?://(archive|security)\.ubuntu\.com/ubuntu@https://mirrors.aliyun.com/ubuntu@g' \
            "$f"; \
    done && \
    if [ -f /etc/apt/sources.list.d/nvidia-l4t-apt-source.list ]; then \
        sed -i -E 's@^[[:space:]]*deb@# deb@' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list; \
    fi && \
    grep -rhE '^[[:space:]]*deb ' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null

# Install ROS 2 apt source and ros-humble-ros-core to pull in system-level
# dependencies (libspdlog1, libtinyxml2-9, python3-packaging, etc.).
# /opt/ros is overridden by a host bind-mount at runtime.
# The ROS apt source uses the Aliyun mirror, reachable from within China.
# The genuine ROS 2 GPG key is fetched by fingerprint F42ED6FBAB17C654 from a keyserver;
# do NOT use mirrors.aliyun.com/ros/ros.key (it lacks the ROS 2 key and causes NO_PUBKEY).
# Before using a keyserver, install dirmngr and create /root/.gnupg (absent in the base
# image), otherwise gpg fails with "No dirmngr".
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl gnupg2 dirmngr lsb-release ca-certificates && \
    mkdir -p /root/.gnupg && chmod 700 /root/.gnupg && \
    echo "standard-resolver" >> /root/.gnupg/dirmngr.conf && \
    echo "disable-ipv6" >> /root/.gnupg/dirmngr.conf && \
    gpg --no-default-keyring --keyring /usr/share/keyrings/ros-archive-keyring.gpg \
        --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-ros-core && \
    apt-get install -y --no-install-recommends \
        usbutils iproute2 tini python3-pip \
        build-essential binutils git \
        python3-opencv python3-numpy \
        libjpeg-dev libpng-dev libtiff-dev \
        libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
        libgtk-3-dev \
        openssh-client \
        libboost-python1.74.0 \
        bluez \
        cmake ninja-build \
        python3-colcon-common-extensions \
        libcurl4-openssl-dev \
        libncurses-dev \
        pkg-config && \
    printf '%s\n' \
        '[global]' \
        'index-url = https://pypi.tuna.tsinghua.edu.cn/simple' \
        'extra-index-url = https://mirrors.aliyun.com/pypi/simple/' \
        '                  https://mirrors.cloud.tencent.com/pypi/simple/' \
        'timeout = 60' \
        'retries = 5' \
        > /etc/pip.conf && \
    pip3 install --no-cache-dir -U pip packaging && \
    pip3 install --no-cache-dir "scikit-build-core<0.10" nanobind && \
    pip3 install --no-cache-dir --no-build-isolation ruckig==0.15.3 && \
    cd /opt && \
    ( timeout 900 git clone --depth 1 -b 4.8.0 https://gitee.com/mirrors/opencv.git opencv-4.8.0 || \
      ( echo "gitee clone failed, falling back to a GitHub mirror in China" && rm -rf /opt/opencv-4.8.0 && \
        curl -fSL --retry 3 --retry-delay 5 -m 1800 -o /tmp/opencv-4.8.0.tar.gz \
             https://gh-proxy.com/https://github.com/opencv/opencv/archive/refs/tags/4.8.0.tar.gz && \
        tar -xzf /tmp/opencv-4.8.0.tar.gz -C /opt && rm -f /tmp/opencv-4.8.0.tar.gz ) ) && \
    cd /opt/opencv-4.8.0 && mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr/local \
          -DBUILD_PYTHON_SUPPORT=OFF \
          -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF \
          -DWITH_GTK=ON \
          -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,photo,video,objdetect \
          -DOPENCV_GENERATE_PKGCONFIG=ON .. && \
    make -j${BUILD_JOBS} && make install && \
    rm -rf /opt/opencv-4.8.0 && \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Auto-source the host-mounted ROS setup in interactive shells.
RUN echo 'if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi' \
        >> /etc/bash.bashrc

# Make the dynamic linker aware of the host-mounted CUDA paths.
RUN echo "/usr/local/cuda/lib64"        > /etc/ld.so.conf.d/cuda.conf && \
    echo "/usr/local/cuda/targets/aarch64-linux/lib" >> /etc/ld.so.conf.d/cuda.conf

LABEL dev.owner="xlq"
LABEL dev.source="https://x2-aimdk.agibot.com/zh-cn/latest/quick_start/container_dev.html"
ENV PATH=/usr/local/cuda/bin:${PATH}
# Load this repository deployment environment explicitly; do not modify a pre-existing project entry.
WORKDIR /workspace

# tini handles PID 1 signal forwarding.
ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["sleep", "infinity"]
