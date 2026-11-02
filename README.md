# X2 Ultra 双臂网页控制源码

更新：2026-11-02。完整使用说明见 [guid.md](guid.md)。

**第一次部署请从 [DEPLOYMENT.md](DEPLOYMENT.md) 开始**：包含Docker/Podman镜像构建、容器创建/复用、独立SDK编译、Conda安装、ROS与DDS配置、只读检查、实机配置、电脑SSH转发、启动和停止命令。配套文件位于 `deploy/jetson/`。

本次同步关节PD、笛卡尔阻抗与拖拽、RGB-D预览和RGB采集、可选YOLO调试，以及腰部运动判断开关。默认 `panel_waist_motion_guard_enabled=false`，保留反馈缺失、过期和非法数据处理。源工程已完成实机接管与预抓取接近，完整双臂连接器抓取尚未成功验证。

适用对象：X2 Ultra、原厂 OmniPicker 及转接件，AimDK 1.1.5 接口。
此副本整理自现有控制工程，保留 Python 源码、网页资源、配置及动力学模型。

## 包含功能

- 双臂 14 关节滑条、每关节 Kp/Kd 调节、当前角度同步、位置保持。
- 左右 OmniPicker 独立开合及力度滑条，0% 闭合、100% 张开。
- 关节位置 PD + 附加重力矩；IMU 修正骨盆倾斜时的重力方向。
- 根据关节反馈估计双臂末端六维力，输出到各自配置的法兰传感器坐标系。
- 无需机器人即可运行的数值演示后端。
- 可切换的笛卡尔阻抗：各臂六维偏移、末端 K/D、重力与关节阻尼；实机使用需要显式启用配置。
- 不接管也实时显示双臂法兰 XYZ、RPY、四元数，以及当前/目标位姿；过期数据明确标记。

## 先在电脑上打开演示

在本目录打开终端，使用 Conda 或兼容工具创建环境：

```bash
conda env create -f environment.yml
conda activate x2-control
python -m x2_kinematics.aimdk_bridge.dashboard --backend demo --mode web
```

浏览器打开 http://127.0.0.1:8765/ 。演示运行在本机，不连接机器人。
这是 Pinocchio 数值演示，不包含三维渲染或接触装配仿真。

## 网页调参和模式切换

- 关节模式：修改每个关节 Kp/Kd，点击“应用 14 关节 PD”。正在运行时平滑过渡，
  不会因编辑输入框而立即改变命令。界面分别显示设定值和当前命令。
- 夹爪：分别设置左右开度和力度，点击“发送开度与力度”；开启实时下发后拖动生效。
  力度对应 SDK 的归一化 `effort` 上限，范围 0～100%，没有标定为实际 N·m。
- 笛卡尔：先点击保持，再选择“笛卡尔阻抗”，点击启动。
  启动时捕获当前法兰参考，ΔXYZ 和旋转向量 ΔRxRyRz 都在该初始法兰局部系下定义。
  平移单位 mm，旋转单位 °；目标经过平移、旋转限速。
- 接管前目标预览跟随当前反馈；启动时实际位姿就是初始目标。
  运行中可点击“以当前实际位姿为新起点”，重新捕获当前反馈，增量归零并平滑接续力矩。
  实时位姿采用 torso_link 下的配置法兰原点，XYZ 以 mm、RPY 以 ° 显示；
  R = Rz(Yaw) Ry(Pitch) Rx(Roll)，四元数顺序 xyzw。位姿显示独立于运动和估力是否已启动。
- 各臂六轴 K/D 可以分别设置；平移刚度单位 N/m，旋转刚度单位 N·m/rad，
  对应阻尼单位 N·s/m 和 N·m·s/rad。
- 拖拽：启动笛卡尔控制后打开独立的“双臂拖拽”开关，保留重力补偿和关节阻尼。
  关闭时以当前实际法兰位姿建立新目标、偏移归零，平滑恢复阻抗，不拉回旧目标。
- `panel_base_motion_guard_enabled=false` 时，骨盆运动指标只显示，不再触发退出或等待静止。
  IMU 数值、姿态范围与新鲜度仍检查；运动中的估力只作诊断，不包含完整基座动力学补偿。
- `panel_waist_motion_guard_enabled=false` 时，腰部速度和角变化不触发运动停机，也不等待腰部静止；
  继续读取编码器更新模型，保留数据有效性、新鲜度和其他关节检查。
- 网页调参只对当前运行进程生效；重启默认值来自 `configs/joint_panel.json`。

笛卡尔控制使用 `Jᵀ` 将末端弹簧与阻尼作用映射成关节力矩，SDK 关节 Kp=0，
附加力矩只加一次重力，并保留单独的关节阻尼。它不使用估计外力作为闭环反馈。
源码副本默认 `panel_cartesian_robot_enabled=false`；接收者按自己的硬件配置启用。源工程的笛卡尔模式、拖拽及运动判断开关已部署调试；本仓库仍保留通用接收配置中的未确认项。
普通保持会捕获并维持当前末端；笛卡尔故障回退撤除任务力矩，只保留重力与关节阻尼，
不能把这种回退理解为可靠的位置锁定或硬件急停。

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
| `x2_kinematics/aimdk_bridge/cartesian_panel_control.py` | 笛卡尔网页控制、目标限速与模式状态 |
| `x2_kinematics/cartesian_impedance.py` | 雅可比转置末端弹簧、阻尼与零空间力矩 |
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


## RGB-D 预览与 RGB 单帧采集

点击“开启预览”后，网页显示头部 RGB-D 的彩色图和深度伪彩色预览；无需启动手臂控制，只读模式也能使用。模拟后端显示带明确水印的测试画面，不代表真实相机已联通。相机在独立进程中订阅和编码，停止预览会退出本项目的相机进程，不操作硬件相机开关。

点击“采集当前 RGB 帧”会保存正在显示的那一帧彩色图，并请求浏览器下载到电脑。运行后端同时留存在 `captures/rgb/`，文件名带北京时间及唯一帧编号。实际电脑下载位置由浏览器决定；也可点击结果中的“重新下载此图”。**不保存深度图，不保存整段视频，不改动原图分辨率、颜色或方向。** 显示帧若已移出缓存会明确报错，不会替换成最新帧。采集目录已加入 Git 忽略规则。可用 `--capture-dir` 指定后端采集目录。

RGB 和深度分别显示各自收到的帧，不表示两路已同步或像素已对齐。深度仅用于伪彩色预览；`16UC1` 按毫米、`32FC1` 按米解释，黑色表示无效深度。预览量程默认 0.15–2 m，频率上限默认 8 Hz，受实际发布频率和网络影响。图片过期或连接断开时标记为过期并禁用采集。

默认订阅 `/aima/hal/sensor/rgbd_head_front/rgb_image/compressed` 与 `/aima/hal/sensor/rgbd_head_front/depth_image`。可在网页配置中通过 `panel_camera_rgb_topic`、`panel_camera_depth_topic` 修改。图像处理依赖 Pillow；ROS 类型来自配套 SDK 环境。源工程已读取实机RGB-D并提取工件点云；接收平台仍需核对自己的话题、编码和参数。


RGB 保存位置：网页增加“RGB 默认保存位置”栏。支持 File System Access API 的 Edge / Chrome 可点击“选择文件夹”，首次选择并授权后，连续采集直接写入该电脑文件夹，不再逐张弹出另存为；不会修改全局浏览器下载设置。网页通过 IndexedDB 记住目录句柄，不保存或猜测电脑绝对路径。重新打开时可能要重新授权一次，目录仍保持所选位置。可点击“改用浏览器下载”清除选择。当前浏览器不支持时明确提示使用 Edge / Chrome，并保留原下载方式。目录拒绝授权、写入失败或同名文件已存在时不会误报保存成功；后端已采集的图仍有下载链接。


## YOLO 电脑视觉预览

新增独立视觉调试页 `http://127.0.0.1:8774/`，原控制页的 RGB-D 栏提供“YOLO 检测页”入口。先建立原控制网页的 SSH 转发并开启相机预览，再在电脑启动检测服务。电脑读取 `/api/camera/state` 及对应帧图像，在电脑 GPU 推理，不修改机器人环境、不提供任何机器人控制接口。

使用可运行 Ultralytics 的 Python 环境：`start_yolo_preview.ps1 -Python <YOLO环境的python.exe> -Model <训练模型best.pt>`。默认读取 `http://127.0.0.1:8765`，可用 `-CameraUrl`、`-Port` 调整。原始图与预测图严格使用同一帧；处理繁忙时取最新帧，不积压图像。界面提供置信度阈值、分割掩码开关、推理耗时、取帧至结果耗时及像素框/轮廓。保存按钮把当前原图、检测图和预测 JSON 一起保存到运行目录的 `reports/yolo_preview/`。

YOLO 依赖是可选项，不加入机械臂的 Pinocchio/SDK 环境。该源码副本不包含个人训练权重或训练数据。当前实现验证过 Ultralytics 8.4.177、YOLO26 实例分割，接收者需使用与自己权重匹配的环境。二维检测/分割结果没有自动变成抓取位姿，也没有用来驱动机械臂；后续使用 RGB-D 时还需核对图像配准、相机内外参及机器人坐标变换。
