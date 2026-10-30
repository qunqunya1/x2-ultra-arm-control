# X2 Ultra 双臂网页控制源码

适用对象：X2 Ultra、原厂 OmniPicker 及转接件，AimDK 1.1.5 接口。
此副本整理自现有控制工程，保留 Python 源码、网页资源、配置及动力学模型。

## 包含功能

- 双臂 14 关节滑条、当前角度同步、位置保持。
- 左右 OmniPicker 独立开合滑条，0% 闭合、100% 张开。
- 关节位置 PD + 附加重力矩；IMU 修正骨盆倾斜时的重力方向。
- 根据关节反馈估计双臂末端六维力，输出到各自配置的法兰传感器坐标系。
- 无需机器人即可运行的数值演示后端。

## 先在电脑上打开演示

在本目录打开终端，使用 Conda 或兼容工具创建环境：

```bash
conda env create -f environment.yml
conda activate x2-control
python -m x2_kinematics.aimdk_bridge.dashboard --backend demo --mode web
```

浏览器打开 http://127.0.0.1:8765/ 。演示运行在本机，不连接机器人。
这是 Pinocchio 数值演示，不包含三维渲染或接触装配仿真。

## 在机器人上运行

将整个文件夹复制到自己的机器人开发目录或容器。
先加载与机器人版本匹配的 ROS2 / AimDK 环境，再激活上述 Python 环境。
该终端中需能同时导入 `pinocchio`、`rclpy`、`aimdk_msgs` 和 `sensor_msgs`。
ROS2 和厂家 SDK 不包含在此副本内；Python 小版本、DDS 域及库路径须与厂家的环境匹配。

先启动只读显示：

```bash
bash run_panel.sh --backend robot --mode observe
```

需要网页控制时，按接收者的硬件核对 `configs/robot.json`：
`actual_onboard_sdk_verified`、`base_assumption_confirmed`、
`effort_feedback_semantics_confirmed` 三项默认均为 `false`，
对应条件核实后设为 `true`，再启动：

```bash
bash run_panel.sh --backend robot --mode web
```

打开网页后，点击启动控制，关节目标从当前反馈角度开始。
启动后端本身不会自动接管手臂或开合夹爪。
接管前由操作者确认原控制器已交还双臂控制，避免多个程序同时下发指令。

在 Windows 电脑另开 PowerShell，建立 SSH 转发；将占位内容换成自己的连接信息：

```powershell
powershell -ExecutionPolicy Bypass -File .\connect_panel.ps1 -RobotHost "用户名@机器人IP"
```

保持该终端打开，在电脑浏览器访问 http://127.0.0.1:8765/ 。
后端绑定机器人回环地址，需使用 SSH 转发；修改端口时两端需一致。

## 配置与源码位置

| 路径 | 内容 |
| --- | --- |
| `configs/robot.json` | SDK 话题、重力补偿及机器人参数 |
| `configs/joint_panel.json` | PD、指令频率、速度、IMU 和夹爪驱动参数 |
| `configs/virtual_flange_sensor.json` | 左右腕部到虚拟传感器的变换与力的符号 |
| `configs/active_robot_model.json` | 动力学模型选择 |
| `x2_kinematics/aimdk_bridge/dashboard.py` | 网页服务、反馈、控制和演示入口 |
| `x2_kinematics/aimdk_bridge/panel_assets/` | HTML / CSS / JavaScript |
| `x2_kinematics/aimdk_bridge/dynamics.py` | 含当前腰部角度的重力、动力学与雅可比 |
| `x2_kinematics/aimdk_bridge/gripper.py` | OmniPicker HAL 读写 |
| `x2_kinematics/aimdk_bridge/async_wrench.py` | 独立估力进程 |
| `x2_kinematics/dual_wrench.py`、`flange_sensor.py` | 关节力矩残差估力与坐标变换 |

`Fx/Fy/Fz` 单位 N，`Mx/My/Mz` 单位 N·m；坐标原点和轴向由左右
`wrist_from_sensor` 配置决定，默认采用名义 OmniPicker 安装变换。
它们随各自末端运动，不是躯干固定坐标轴。末端到训练传感器的实际变换、符号和零偏需另行对齐。

当前估力依赖准静态受支撑骨盆、近似静止腰部及正确的关节力矩反馈语义。
IMU 用于修正重力方向，不包含悬挂大幅摆动的基座加速度动力学。
估计值受模型、摩擦和姿态可观测性影响，尚不能作为已验证的真实六维传感器替代输入。

模型保留关节、坐标变换、质量和惯量，移除了显示网格及碰撞几何，
不提供避碰或三维可视化；手指在动力学模型中固定，实机夹爪另经 HAL 控制。
模型由厂家不同版本资源组合修改，末端惯量未进行实机辨识；来源及许可见 `models/NOTICE.txt`、`models/LICENSE`。

此副本不含完整 SDK、依赖环境、日志、历史实验、测试报告、截图、个人 IP、账号或凭据。
