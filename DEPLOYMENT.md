# 从容器创建到实机网页控制

更新：2026-11-02。先读本页完成部署，再看 [guid.md](guid.md) 操作网页。

## 0. 内容与验证边界

本仓库提供双臂关节控制、笛卡尔阻抗、拖拽、左右夹爪控制、六维估力、RGB-D预览和可选电脑YOLO页。它不包含厂家SDK、固件、训练权重、完整装配/抓取研究工程，也不提供网页控制的避碰世界。

已用环境为Jetson ARM64、Ubuntu22.04/L4T R36、ROS2 Humble、与机器人1.1.5配套的AimDK、独立Podman容器。Python3.10、Pinocchio4.1.0、NumPy2.2.6、SciPy1.15.2的ROS/SDK导入与实机控制在源工程中验证过。

本页新增的部署目录基于该环境配方整理，完成本机语法和逻辑检查；**本次没有重新构建镜像或在机器人上从零部署。Docker是对应运行方式，尚未在本项目实机独立验证。** 不同固件、SDK、JetPack版本需要重新匹配，不能只换镜像标签就认为兼容。

本页明确区分三种终端：**电脑PowerShell、机器人宿主机Bash、容器内Bash**。容器引擎安装与管理员权限是前提；不在共享机器人上自动安装引擎、重装驱动或修改其他人的环境。

## 1. 宿主机：选择引擎、建立自己的工作目录

电脑PowerShell先连接自己的机器人账号：

```powershell
ssh <user>@<robot-ip>
```

下面在机器人宿主机Bash执行。实际环境使用Podman；若选择Docker，只将第一行改为 `export ENGINE=docker`，后续一直使用同一引擎和同一账号，不要交替使用普通Docker、sudo Docker及Podman。

```bash
set -euo pipefail
export ENGINE=podman
export DEV_ROOT="$HOME/xlq-dev"
export NAME=x2-xlq-dev
export IMAGE=localhost/x2-xlq-dev:web-20261102
export SOURCE_DIR="$DEV_ROOT/projects/x2-web-control"
export CPUSET=6,7

uname -m
cat /etc/os-release
cat /etc/nv_tegra_release
"$ENGINE" version
ip -brief -4 addr

mkdir -p "$DEV_ROOT/projects" "$DEV_ROOT/home" "$DEV_ROOT/logs" "$DEV_ROOT/config"
test ! -e "$SOURCE_DIR"
git clone https://github.com/qunqunya1/x2-ultra-arm-control.git "$SOURCE_DIR"
cd "$SOURCE_DIR"
```

`CPUSET=6,7` 对应源工程后来使用的两个核心。用 `lscpu -e=CPU,ONLINE` 检查它们存在，再结合负载选择本机两个可用核心；6/7不适用于所有机器人。后续编译与后端命令显式使用该值。

如果源码目录已经存在，跳过 `test` 和 `git clone`，先 `git status` 检查本地配置，核对后再更新。不要覆盖已有修改。新终端需要重新设置上述变量。

## 2. 宿主机：复制匹配版本SDK

SDK来自机器人厂家随固件提供的位置。下面默认原件在当前账号的 `~/aimdk`；若不在此处，修改 `VENDOR_SDK`。只读取原件，编译和修改均在独立副本里进行。

```bash
export VENDOR_SDK="$HOME/aimdk"
test -d "$VENDOR_SDK/src"

if [ -d "$DEV_ROOT/sdk/src" ]; then
    echo 'Own SDK already exists; verify its release instead of overwriting it.'
else
    test ! -e "$DEV_ROOT/sdk"
    mkdir "$DEV_ROOT/sdk"
    cp -a "$VENDOR_SDK/src" "$DEV_ROOT/sdk/src"
fi
```

本工程需要消息包、Python库及配套C++依赖完整。厂家SDK如果还依赖 `src` 之外的资源，应根据对应版本的工程说明一并复制到自己的SDK目录。不要复制原件中带旧绝对路径的 `build/install/log` 当作新副本的编译结果。

## 3. 宿主机：构建镜像

仓库附带的 [Dockerfile](deploy/jetson/Dockerfile) 来自已用的Jetson配方，安装的是容器内系统依赖，编译并行度默认2。它使用L4T R36.2.0基础层；ROS运行目录稍后从机器人只读挂载。

```bash
cd "$SOURCE_DIR"
"$ENGINE" build --network=host --build-arg BUILD_JOBS=2 \
    -t "$IMAGE" -f deploy/jetson/Dockerfile .
"$ENGINE" image inspect "$IMAGE" --format '{{index .Config.Labels "dev.owner"}}'
```

最后一行应为 `xlq`。如果本项目容器已经创建且仍准备复用它，可跳过构建；新建镜像不会改变已有容器。不要删除、重建现有容器来“应用”一个新镜像。

Dockerfile中的OpenCV、ROS和系统包安装可能较慢。构建失败时查看具体下载或编译错误；本指南不执行宿主机全局升级或全局清理。

## 4. 宿主机：创建或核验容器

下面两种引擎使用相同的基本参数。网页控制依靠CPU和ROS通信，默认不请求GPU。YOLO在电脑独立运行；本节不需要为它修改机器人GPU配置。

先核对厂家ROS目录，再创建或检查专属容器：

```bash
test -f /opt/ros/humble/setup.bash
DEV_ROOT=$(cd "$DEV_ROOT" && pwd -P)

if "$ENGINE" container inspect "$NAME" >/dev/null 2>&1; then
    test "$("$ENGINE" inspect --format '{{index .Config.Labels "dev.owner"}}' "$NAME")" = xlq
    MOUNT_ROOT=$("$ENGINE" inspect --format '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}' "$NAME")
    test "$MOUNT_ROOT" = "$DEV_ROOT"
    echo 'Existing owned container verified; not recreated.'
else
    "$ENGINE" image inspect "$IMAGE" >/dev/null
    RUNTIME_MOUNTS=()
    if [ -d /usr/local/cuda ]; then
        RUNTIME_MOUNTS+=(-v /usr/local/cuda:/usr/local/cuda:ro)
    fi
    # Firmware ROS CMake exports may contain absolute NumPy include paths.
    while IFS= read -r HEADER_DIR; do
        if [ -d "$HEADER_DIR" ]; then
            RUNTIME_MOUNTS+=(-v "$HEADER_DIR:$HEADER_DIR:ro")
        fi
    done < <(grep -rhoE '/[A-Za-z0-9_./+-]*(dist|site)-packages/[A-Za-z0-9_./+-]*include' \
        --include='*.cmake' /opt/ros/humble/share 2>/dev/null | sort -u)
    "$ENGINE" run -d --name "$NAME" --label dev.owner=xlq \
        --network=host --ipc=private --memory=6g \
        -e HOME=/home/dev -e ROS_DOMAIN_ID=0 \
        -e LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/host-libs/aarch64-linux-gnu \
        -v "$DEV_ROOT:/workspace:rw" \
        -v "$DEV_ROOT/home:/home/dev:rw" \
        -v /opt/ros:/opt/ros:ro \
        -v /usr/lib/aarch64-linux-gnu:/host-libs/aarch64-linux-gnu:ro \
        -v /etc/machine-id:/etc/machine-id:ro \
        "${RUNTIME_MOUNTS[@]}" \
        -w /workspace "$IMAGE"
fi

"$ENGINE" inspect --format '{{.Name}} running={{.State.Running}} owner={{index .Config.Labels "dev.owner"}}' "$NAME"
```

已有同名容器的所有者或挂载检查失败时，停止本段操作并核对资源，不能直接继续下一步。`inspect`失败也可能是连接引擎或权限问题，应先解决该问题。

`/workspace`及独立HOME是本项目的可写目录；厂家ROS、系统共享库及CMake导出的NumPy头文件目录只读。已有CUDA目录也只读挂载以满足SDK编译引用，这不代表已开放GPU设备。这里使用 `--network=host`，不额外添加 `-p 8765:8765`。网页仍只绑定回环地址，通过SSH转发访问。

需要扩展GPU计算时，先核对现有NVIDIA运行时。Podman的CDI参数为 `--device=nvidia.com/gpu=all`；Docker可使用已配置的NVIDIA运行时。两者配置不同，不要把Podman的启动参数直接复制给旧版Docker，也不要为运行网页控制而重装宿主机驱动。详见文末官方参考。

## 5. 宿主机：启动并进入容器

```bash
test "$("$ENGINE" inspect --format '{{index .Config.Labels "dev.owner"}}' "$NAME")" = xlq
"$ENGINE" start "$NAME"
"$ENGINE" exec -it -w /workspace "$NAME" taskset -c "$CPUSET" bash
```

`start`和`exec`只是启动容器、打开终端，不自动运行机械臂控制。下面第6～10节均在这个容器终端内执行。

## 6. 容器内：设置DDS和编译SDK

从宿主机 `ip -brief -4 addr` 的结果选择与机器人SDK通信的网卡IPv4，替换下面占位值。这不是电脑IP，也不一定是SSH所用Wi-Fi接口。

```bash
cd /workspace/projects/x2-web-control
export ROBOT_LAN_IP='<robot-sdk-interface-ipv4>'
/usr/bin/python3 deploy/jetson/make_dds_profile.py \
    --interface "$ROBOT_LAN_IP" --output /workspace/config/dds-panel.xml
source deploy/jetson/env.sh

cd /workspace/sdk
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 \
    colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

DDS模板只使用UDP并限定该网卡，避免共享环境中的Fast DDS共享内存锁冲突。输出文件独占创建，已有文件不会被覆盖；复用环境时跳过生成，核对它的地址即可。`ROS_DOMAIN_ID`应与厂家节点一致，当前默认0。

确认本次SDK构建成功后再继续。后续每个新终端都需要 `source deploy/jetson/env.sh`，否则仅“容器已启动”不代表ROS/SDK环境已经加载。

## 7. 容器内：安装独立Python环境

```bash
cd /workspace/projects/x2-web-control
bash deploy/jetson/install-miniconda.sh
/home/dev/miniconda3/bin/conda env create -f environment.yml
```

安装器固定ARM64 Miniconda版本并验证SHA256，依赖进入本项目的持久HOME；不覆盖未知的已有安装。`environment.yml`创建 `x2-control`，固定Python3.10、Pinocchio4.1.0、NumPy2.2.6、SciPy1.15.2，另外安装Pillow。

已有相同环境时，不要重复 `env create`；先检查 `conda env list` 和下一节导入检查。源工程已有环境名 `xlq-arm-115`，确认兼容后可通过 `export X2_CONDA_ENV=xlq-arm-115` 复用它；新部署默认使用 `x2-control`。如果Conda本身装在别处，用 `X2_CONDA_EXE` 指定其可执行文件路径。

## 8. 容器内：只读依赖与SDK检查

```bash
cd /workspace/projects/x2-web-control
source deploy/jetson/env.sh
/home/dev/miniconda3/bin/conda run --no-capture-output -n x2-control \
    python -B deploy/jetson/check_imports.py
```

预期导入NumPy、SciPy、Pinocchio、Pillow、rclpy、SDK和图像/IMU消息，校验消息字段并往返序列化。输出应包含 `message_serialization: true`、`ros_initialized: false`、`robot_commands_sent: 0`。它不初始化ROS节点，不发布关节或夹爪指令。

若 `rclpy` 不可导入，先检查ROS是否source及Python小版本是否匹配；若SDK消息不可导入，检查独立SDK的构建和 `install/local_setup.bash`。不要直接向厂家系统Python全局安装另一套NumPy。`GLIBCXX` 等错误需要核对Conda/ROS的共享库解析，不通过覆盖宿主机库解决。

## 9. 容器内：启动只读网页

```bash
cd /workspace/projects/x2-web-control
bash deploy/jetson/run_backend.sh --backend robot --mode observe --port 8765
```

保持这个终端运行。后端绑定容器/宿主机回环地址，先不要点击任何控制操作。新建另一个**电脑PowerShell**终端：

```powershell
git clone https://github.com/qunqunya1/x2-ultra-arm-control.git
cd x2-ultra-arm-control
.\connect_panel.ps1 -RobotHost '<user>@<robot-ip>' -Port 8765
```

电脑已有源码时跳过克隆，直接进入目录启动转发。保留转发终端，浏览器打开 <http://127.0.0.1:8765/>。也可直接使用OpenSSH：

```powershell
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 <user>@<robot-ip>
```

网页应有新鲜关节、法兰位姿、IMU、估力和夹爪反馈。只读模式禁止接管。可在电脑检查：

```powershell
$s=Invoke-RestMethod 'http://127.0.0.1:8765/api/state'
$s | Select-Object backend,mode,control_state,fault,motor_commands_sent,hardware_command_publisher_created
```

预期 `backend=robot`、`mode=observe`，硬件指令计数为0，硬件发布器未创建。估力可以有诊断提示；模型尚未标定的数值不是已验证的六维力测量。

## 10. 容器内：建立本机配置并启用实机网页

先在第9节后端终端按Ctrl+C退出只读后端。不要在同一端口启动两个后端。

先建立本机配置副本，保留仓库默认设置：

```bash
cd /workspace/projects/x2-web-control
test ! -e configs/robot.local.json
test ! -e configs/joint_panel.local.json
cp configs/robot.json configs/robot.local.json
cp configs/joint_panel.json configs/joint_panel.local.json
```

已有副本时跳过复制。以下配置在文本编辑器中修改；不要只为消除提示而把未核实的物理条件设为true：

| 文件和字段 | 何时设置 |
| --- | --- |
| `robot.local.json: actual_onboard_sdk_verified` | 消息与当前固件SDK检查通过后设true |
| `robot.local.json: base_assumption_confirmed` | 骨盆/躯干支撑与重力假设在现场核实后设true |
| `robot.local.json: effort_feedback_semantics_confirmed` | 关节effort语义已核对后设true |
| `joint_panel.local.json: panel_cartesian_robot_enabled` | 准备启用实机笛卡尔模式时设true |

关节PD、笛卡尔K/D、速度和力矩参数在 `joint_panel.local.json`；工具坐标在 `virtual_flange_sensor.json`。本仓库的刚度和增益是项目调试值，应根据本机末端、负载和支撑核对。两份 `.local.json` 被Git忽略，不会因普通源码更新被提交到仓库。

当前网页启动还会检查其他指令发布器和反馈条件。操作者需先确认厂家原控制器已交还双臂控制，本程序不调用厂家服务自动停止原生MC，也不提供原生MC交接指令。`control_ownership_confirmed` 是旧独立入口的兼容字段，不是当前网页交接的证明。

真正启动可接管的网页后端：

```bash
bash deploy/jetson/run_backend.sh --backend robot --mode web --port 8765 \
    --config configs/robot.local.json --panel-config configs/joint_panel.local.json
```

电脑沿用第9节转发。启动进程本身不会自动接管；先看网页“启动条件检查”，全部满足后点击“启动实机控制”，从当前实际位姿接管。先关节模式，再按需要保持并切换笛卡尔模式；夹爪命令单独发送。

后台配置合并顺序是 `--config` 的机器人设置，再由 `--panel-config` 的网页参数覆盖；修改文件不会热更新运行进程。需重启时先可靠支撑双臂，重启会短暂停止重力补偿。参数含义和拖拽操作见 [guid.md](guid.md)。

## 11. 日常再次启动：不再重复构建和安装

机器人宿主机新终端设置第1节引擎和目录变量，核对所属标签后：

```bash
test "$("$ENGINE" inspect --format '{{index .Config.Labels "dev.owner"}}' "$NAME")" = xlq
"$ENGINE" start "$NAME"
"$ENGINE" exec -it -w /workspace/projects/x2-web-control "$NAME" \
    taskset -c "$CPUSET" bash deploy/jetson/run_backend.sh \
    --backend robot --mode web --port 8765 \
    --config configs/robot.local.json --panel-config configs/joint_panel.local.json
```

电脑PowerShell启动 `connect_panel.ps1`，打开8765网页即可。只读诊断把 `--mode web` 换成 `--mode observe`。

已有原工程容器与源码时，也可以继续使用原来的工程启动器；不要将共享仓库的默认未确认配置直接覆盖原机载live配置。

## 12. 停止、更新和常见问题

停止运动时先在网页保持并可靠支撑双臂；在**后端终端**按Ctrl+C正常退出。网页“保持”仍可能发布保持指令，不能等同于断电或撤销固件上次指令。退出**SSH转发终端**只关闭转发，不保证退出独立后端。

确认本项目后端已经退出，再在**宿主机**停止且仅停止自己的容器：

```bash
test "$("$ENGINE" inspect --format '{{index .Config.Labels "dev.owner"}}' "$NAME")" = xlq
"$ENGINE" stop --time 15 "$NAME"
"$ENGINE" inspect --format '{{.State.Running}}' "$NAME"
```

最后应输出 `false`。数据保留在自己的工作目录和HOME中。不得用全局 `stop/prune` 或删除他人容器解决本项目问题。

源码更新先 `git status`，保存自己的改动，再 `git pull --ff-only`。核对本机配置与新增字段，正常重启自己的后端。新依赖确有需要时才更新本项目独立环境，不升级宿主机或其他容器。

| 问题 | 检查项 |
| --- | --- |
| 无法拉取基础镜像 | Dockerfile的镜像地址、ARM64平台和网络；不要换用不匹配L4T |
| 容器已在运行但导入失败 | 第8节四类依赖；容器内是否source正确ROS/SDK和使用Python3.10 |
| ROS没有反馈 | `ROS_DOMAIN_ID`、DDS网卡IPv4、host网络、机器人厂家话题是否存在 |
| Fast DDS共享内存锁错误 | 是否source本项目UDP配置；不删除他人共享内存文件 |
| 页面打不开 | 远端后端是否运行、本地转发是否在运行、两个端口是否一致 |
| 端口占用 | 是否本项目已有后端，优先复用；需要时两端统一换端口 |
| 启动按钮拒绝 | 逐项启动检查、当前控制归属、关节范围、反馈年龄和原控制器交接 |
| 腰部速度误报 | 回读 `waist_feedback.motion_guard_enabled`；看本次加载的网页配置 |
| 参数改了没有变化 | 核对 `--config/--panel-config` 路径和是否正常重启 |
| 实际动作跟随不好 | 检查负载/控制增益和CPU负载；不要根据未标定估力直接认定接触 |

## 官方参考

- [AimDK容器开发说明](https://x2-aimdk.agibot.com/zh-cn/latest/quick_start/container_dev.html)：本项目原始容器配方的来源；具体文件按本仓库保存版本说明。
- [Docker host网络](https://docs.docker.com/engine/network/drivers/host/)：Linux宿主网络模式及端口映射行为。
- [NVIDIA CDI](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/cdi-support.html)：Podman设备访问的前提和参数。
- [NVIDIA容器运行示例](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html)：Docker与Podman的不同GPU启动方式；须以设备已配置环境为前提。
