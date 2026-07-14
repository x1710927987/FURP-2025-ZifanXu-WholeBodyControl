# WSL 脚本说明

本目录存放 **WSL2 + ROS 2** 环境下的辅助脚本，按功能分子目录：

```
scripts/
├── README.md
├── berxel/
│   ├── camera/          # USB 挂载、预览、快照
│   ├── calibration/     # 棋盘格内参标定
│   └── handeye/         # AprilTag 手眼标定（含 auto / auto_fast）
├── roarm/               # RoArm-M3 驱动与 MoveIt
└── wsl/                 # Panda MoveIt 仿真（Jazzy）
```

所有脚本均应在 **WSL 终端**内运行（PowerShell 脚本除外），不要从 Windows PowerShell 直接调用 `ros2 run`（参数引号会被截断）。

## 前置条件

- WSL2 Ubuntu 24.04，已安装 ROS 2 Jazzy
- 工作空间 `~/ros2_ws` 已按 [`wsl/setup_moveit.sh`](#wslsetup_moveitsh) 配置
- 建议在 `~/.bashrc` 中加入：

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb
source ~/ros2_ws/install/local_setup.bash
```

从仓库根目录调用脚本的示例路径：

```bash
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/<脚本名>.sh
```

---

## 按场景使用

### 场景 1：首次配置环境（只需运行一次）

**目标：** 安装依赖、克隆 `panda_gz_moveit2` + `pymoveit2`、打 Jazzy 补丁并编译。

| 脚本 | 说明 |
|------|------|
| [`wsl/setup_moveit.sh`](#wslsetup_moveitsh) | 一键完成上述全部步骤（需要 `sudo` 密码） |

```bash
bash scripts/wsl/setup_moveit.sh
```

---

### 场景 2：验证 MoveIt 2 机械臂节点能否正常运行

**目标：** 确认 `move_group`、控制器、`pymoveit2` 关节运动链路畅通。这是最基本的冒烟测试。

**步骤：**

1. **终端 1** — 启动 RViz + MoveIt（fake 硬件，无 Gazebo）：

```bash
bash scripts/wsl/launch_fake_control.sh
```

看到 `Stack ready` 且无持续 `FATAL` 即可。

2. **终端 2** — 发送关节空间目标，让机械臂动起来：

```bash
bash scripts/wsl/test_joint_goal.sh
```

**期望结果：** 终端输出 `Moving to {joint_positions: ...}`，RViz 中 Panda 机械臂运动。

| 脚本 | 作用 |
|------|------|
| [`wsl/launch_fake_control.sh`](#wsllaunch_fake_controlsh) | 启动 fake control 栈（含延迟 spawn 控制器，修复 Jazzy 竞态） |
| [`wsl/test_joint_goal.sh`](#wsltest_joint_goalsh) | 调用 pymoveit2 内置 `ex_joint_goal.py` 做关节目标测试 |

---

### 场景 3：验证末端笛卡尔规划（位姿目标）

**目标：** 给定末端 `(x, y, z)` 与姿态，由 MoveIt 规划并执行。

**前提：** 场景 2 的终端 1 已运行 `wsl/launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl/test_pose_goal.sh
```

**期望结果：** 输出 `Moving to {position: ...}`，RViz 中末端执行器朝目标位姿运动。

| 脚本 | 作用 |
|------|------|
| [`wsl/test_pose_goal.sh`](#wsltest_pose_goalsh) | 单次笛卡尔位姿目标（`ex_pose_goal.py`） |

---

### 场景 4：Gazebo 物理仿真下运行 MoveIt

**目标：** 在 Gazebo Harmonic 中加载 Panda，配合 MoveIt 控制。

**终端 1：**

```bash
bash scripts/wsl/launch_gz_control.sh
```

在 Gazebo 窗口点击 **Play**，再于 **终端 2** 运行场景 2 或 3 的测试脚本。

| 脚本 | 作用 |
|------|------|
| [`wsl/launch_gz_control.sh`](#wsllaunch_gz_controlsh) | 启动 `ex_gz_control.launch.py`（Gazebo + RViz + MoveIt） |

---

### 场景 5：避障规划（碰撞物体 + 重规划）

**目标：** 向规划场景添加障碍球体，再尝试末端位姿规划。

**前提：** 终端 1 已运行 `wsl_launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl/test_collision_planning.sh
```

流程：添加球体 → 规划 EE 位姿 → 尝试清空场景。

**说明：** `panda_gz_moveit2` 的 URDF 缺少 link 碰撞网格，碰撞感知规划可能返回 `FAILURE`；添加障碍物体到场景仍会成功，可在 RViz 中看到。清空场景需 `pip3 install trimesh`。

| 脚本 | 作用 |
|------|------|
| [`wsl/test_collision_planning.sh`](#wsltest_collision_planningsh) | `ex_collision_primitive.py` + `ex_pose_goal.py` + `ex_clear_planning_scene.py` |

---

### 场景 6：一次性跑完全部末端测试

**目标：** 关节、位姿、夹爪、障碍、带障重规划串联执行。

**前提：** 终端 1 已运行 `wsl_launch_fake_control.sh`。

**终端 2：**

```bash
bash scripts/wsl/run_ee_tests.sh
```

| 脚本 | 作用 |
|------|------|
| [`wsl/run_ee_tests.sh`](#wslrun_ee_testssh) | 批量运行 pymoveit2 官方示例（5 项测试） |

---

## 脚本速查表

| 脚本 | 场景 | 需要几个终端 | 依赖 launch |
|------|------|-------------|-------------|
| `wsl/setup_moveit.sh` | 首次安装 | 1 | 无 |
| `wsl/launch_fake_control.sh` | 启动 fake 栈 | 1（保持运行） | 自包含 |
| `wsl/launch_gz_control.sh` | 启动 Gazebo 栈 | 1（保持运行） | 自包含 |
| `wsl/test_joint_goal.sh` | 关节运动验证 | 2 | fake 或 gz |
| `wsl/test_pose_goal.sh` | 末端位姿验证 | 2 | fake 或 gz |
| `wsl/test_collision_planning.sh` | 避障规划 | 2 | fake 或 gz |
| `wsl/run_ee_tests.sh` | 全套 EE 测试 | 2 | fake 或 gz |
| `berxel/camera/attach_camera.ps1` | 附加 iHawk 相机 USB | 1（管理员 PS） | 无 |
| `berxel/camera/deploy_calib.sh` | 初始化标定数据目录 | 1 | 无 |
| `berxel/camera/preview.sh` | RGB 实时预览 | 1 | 相机已 attach |
| `berxel/camera/snapshot.sh` | 单帧测试截图 | 1 | 相机已 attach |
| `berxel/calibration/collect.sh` | 采集棋盘格标定图 | 1 | 相机已 attach |
| `berxel/calibration/run.sh` | 计算相机内参 | 1 | 已采集图像 |
| `berxel/handeye/collect.sh` | AprilTag 手眼采样（手动） | 1 | 相机 + RoArm 已启动 |
| `berxel/handeye/auto.sh` | 手眼全自动（含 joint_states 约束） | 1 | 相机 + RoArm 已启动 |
| `berxel/handeye/auto_fast.sh` | 手眼全自动（快速，仅 pose 约束） | 1 | 相机 + RoArm 已启动 |
| `berxel/handeye/solve.sh` | 手眼标定解算 | 1 | 已采集 hand_eye 样本 |
| `berxel/handeye/move_to_tag.sh` | 检测 AprilTag → 移动 TCP | 1 | 外参已标定 + 相机 + RoArm |
| `berxel/handeye/grasp_apriltag.sh` | Tag 两段式接近（悬停再下降） | 1 | 外参已标定 + 相机 + RoArm |

---

## 各脚本详细说明

### `wsl/setup_moveit.sh`

- **用途：** 新机器或重装后，配置 MoveIt + pymoveit2 工作空间。
- **操作：** `apt` 安装 ROS 包 → 克隆 `panda_gz_moveit2`（jazzy 分支）与 `pymoveit2` → 将 `fake_components` 改为 `mock_components` → `colcon build`。
- **注意：** 需要 `sudo`；`rosdep install` 失败时可忽略，以 `colcon build` 结果为准。

### `wsl/launch_fake_control.sh`

- **用途：** 启动 `ex_fake_control.launch.py`（RViz + move_group + mock ros2_control）。
- **操作：** 清理残留进程 → 后台 launch → 等待 15s → 手动 spawn 三个控制器。
- **日志：** `/tmp/panda_fake_control.log`
- **注意：** 脚本结束后 launch 仍在后台；再次运行前可先 `killall ros2_control_node move_group rviz2`。

### `wsl/launch_gz_control.sh`

- **用途：** 启动 `ex_gz_control.launch.py`（Gazebo + MoveIt）。
- **操作：** 前台运行，Ctrl+C 结束。
- **注意：** 需在 Gazebo 中点击 Play 后，仿真时间才会推进。

### `wsl/test_joint_goal.sh`

- **用途：** 验证 pymoveit2 关节空间规划与执行。
- **底层命令：** `ros2 run pymoveit2 ex_joint_goal.py`
- **典型用途：** 判断 MoveIt 2 自带（经 pymoveit2 调用的）机械臂控制链路是否正常。

### `wsl/test_pose_goal.sh`

- **用途：** 验证末端笛卡尔位姿规划。
- **底层命令：** `ros2 run pymoveit2 ex_pose_goal.py`
- **默认目标：** position `[0.25, 0.0, 1.0]`，四元数 `[0, 0, 0, 1]`。

### `wsl/test_collision_planning.sh`

- **用途：** 练习避障规划流程。
- **步骤：** 添加球体障碍 → EE 位姿规划 → 清空规划场景。
- **可选依赖：** `pip3 install trimesh`（用于 `ex_clear_planning_scene.py`）。

### `wsl/run_ee_tests.sh`

- **用途：** 连续运行 5 项 pymoveit2 官方示例（位姿 ×2、夹爪、障碍、带障位姿）。
- **耗时：** 约 1–2 分钟。

---

## 推荐练习顺序

```
wsl/setup_moveit.sh          # 仅首次
    ↓
wsl/launch_fake_control.sh   # 终端 1
    ↓
wsl/test_joint_goal.sh       # 终端 2：先确认能动
    ↓
wsl/test_pose_goal.sh        # 末端笛卡尔
    ↓
wsl/test_collision_planning.sh
    ↓
wsl/launch_gz_control.sh     # 再上 Gazebo
```

---

## RoArm-M3 实体机械臂（独立 WSL 发行版 `RoArm_M3`）

与 Xenon/Jazzy/Panda **仿真** 分开。教程：[微雪 RoArm-M3 ROS2](https://docs.waveshare.net/RoArm-M3/ROS2-tutorial)

| 项目 | 路径 / 说明 |
|------|-------------|
| WSL 发行版 | `RoArm_M3`（Ubuntu 22.04 + Humble） |
| 安装位置 | `D:\WSL\RoArm_M3` |
| Linux 用户 | `ag`（镜像自带；`~/.bashrc` 已配置 ROS） |
| 工作区 | `~/roarm_ws/roarm_ws-ros2-humble` |
| 环境变量 | `export ROARM_MODEL=roarm_m3` |

### 场景 R1：每次开机后挂载 USB 串口

机械臂上电 + USB 接好后，在 **管理员 PowerShell**：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/roarm/attach_usb.ps1
```

在 **RoArm_M3** 终端确认：`ls /dev/ttyUSB0`

### 场景 R2：启动驱动（终端 1，保持运行）

```bash
wsl -d RoArm_M3
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/roarm/launch_driver.sh
```

**警告：** 连接后机械臂可能回零/运动，周围保持空旷。

### 场景 R3：RViz 关节滑块 或 MoveIt2 末端规划（终端 2）

```bash
wsl -d RoArm_M3
# 关节滑块
bash .../scripts/roarm/launch_display.sh
# 或 MoveIt2 拖拽末端
bash .../scripts/roarm/launch_moveit.sh
```

### 场景 R4：键盘控制机械臂（MoveIt Servo）

教程：[键盘控制机械臂](https://docs.waveshare.net/RoArm-M3/ROS2-tutorial)。先结束 display / moveit 例程。

**终端 1** — 驱动（保持运行）：

```bash
bash .../scripts/roarm/launch_driver.sh
```

**终端 2** — Servo 栈（保持运行）：

```bash
bash .../scripts/roarm/launch_servo.sh
```

**终端 3** — 键盘控制（**保持此窗口焦点**，按 `Q` 退出）：

```bash
bash .../scripts/roarm/keyboard_control.sh
```

### 场景 R5：指令控制机械臂

教程：[指令控制机械臂](https://docs.waveshare.net/RoArm-M3/ROS2-tutorial)。先结束其它控制例程。

**终端 1** — 驱动：

```bash
bash .../scripts/roarm/launch_driver.sh
```

**终端 2** — 指令控制栈：

```bash
bash .../scripts/roarm/launch_command_control.sh
```

**终端 3** — 服务/话题示例（可只跑单项）：

```bash
bash .../scripts/roarm/run_command_examples.sh all
# 或: pose | joint | line | gripper | led | circle
```

### 场景 R6：MoveIt MTC 规划轨迹

教程：[MoveIt MTC 规划轨迹](https://docs.waveshare.net/RoArm-M3/ROS2-tutorial)。一次只运行一个 `run.launch.py`。

**终端 1** — 驱动：

```bash
bash .../scripts/roarm/launch_driver.sh
```

**终端 2** — MTC 演示界面：

```bash
bash .../scripts/roarm/launch_mtc_demo.sh
```

**终端 3** — 选择一种演示（切换前 Ctrl+C 结束上一个）：

```bash
bash .../scripts/roarm/run_mtc.sh cartesian
bash .../scripts/roarm/run_mtc.sh cartesian_modular
bash .../scripts/roarm/run_mtc.sh pick_place
```

在 MTC 界面选中完整路径后点击 **Exec**。

| 脚本 | 作用 |
|------|------|
| `roarm/attach_usb.ps1` | Windows：usbipd 绑定/挂载 CP2102 串口 |
| `roarm/launch_driver.sh` | `roarm_driver` 驱动节点 |
| `roarm/launch_display.sh` | RViz + Joint State Publisher |
| `roarm/launch_moveit.sh` | MoveIt2 + RViz MotionPlanning |
| `roarm/launch_servo.sh` | MoveIt Servo（键盘控制前置，终端 2） |
| `roarm/keyboard_control.sh` | `keyboardcontrol` 键盘点动（终端 3） |
| `roarm/launch_command_control.sh` | 指令控制 launch（终端 2） |
| `roarm/run_command_examples.sh` | 位姿/直线/夹爪/LED/画圆等服务示例（终端 3） |
| `roarm/launch_mtc_demo.sh` | MTC 演示 UI（终端 2） |
| `roarm/run_mtc.sh` | MTC 子演示 `cartesian` / `cartesian_modular` / `pick_place`（终端 3） |

---

## Berxel iHawk100Q 相机内参标定（Ubuntu-24.04 WSL）

与 RoArm/Panda **分开**。相机通过 **usbipd** 附加到默认 WSL `Ubuntu-24.04`（不是 `RoArm_M3`）。

| 项目 | 说明 |
|------|------|
| 相机型号 | Berxel iHawk100 (VID:PID 0603:000a) |
| RGB 设备 | `/dev/video0`（UVC MJPEG） |
| 标定分辨率 | 640×400 |
| 棋盘格 | 8×6 内角点，方格 25 mm |
| 标定数据 | `calibration/berxel/` |
| 脚本 | `scripts/berxel/` |

### 场景 B1：附加 USB 相机（每次开机，管理员 PowerShell）

```powershell
powershell -ExecutionPolicy Bypass -File scripts/berxel/camera/attach_camera.ps1
# 或: usbipd attach --wsl --busid 2-1
```

WSL 内验证：

```bash
wsl -d Ubuntu-24.04
ls /dev/video0
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/camera/deploy_calib.sh
```

### 场景 B2：采集标定图像（固定棋盘格，移动相机）

将棋盘格贴平固定在墙面/桌面，移动相机覆盖不同角度与距离。目标 **15~25** 张。

```bash
wsl -d Ubuntu-24.04
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/calibration/collect.sh
# 可选自动保存: .../berxel/calibration/collect.sh --auto
```

窗口操作：`S` 保存（仅角点检测成功时），`Q` 退出。图像保存到 `calibration/berxel/calib_images/`。

### 场景 B3：计算内参

```bash
wsl -d Ubuntu-24.04
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/calibration/run.sh
```

输出：

| 文件 | 内容 |
|------|------|
| `calibration/berxel/intrinsics.yaml` | 内参矩阵、畸变系数、RMS 误差 |
| `calibration/berxel/calib_report.txt` | 每张图检测状态 |
| `calibration/berxel/undistort_sample.jpg` | 去畸变对比样图 |

验收：有效图像 ≥ 15，RMS < 0.5 px（可接受 < 1.0 px）。

| 脚本 | 作用 |
|------|------|
| `berxel/camera/attach_camera.ps1` | Windows：usbipd 绑定/挂载 Berxel 相机 |
| `berxel/camera/deploy_calib.sh` | 初始化 `calibration/berxel/` 目录与依赖 |
| `berxel/camera/preview.sh` | RGB 实时预览 |
| `berxel/camera/snapshot.sh` | 单帧测试截图 |
| `berxel/calibration/collect.py` / `.sh` | 交互采集棋盘格图像 |
| `berxel/calibration/run.py` / `.sh` | OpenCV 棋盘格内参标定 |

### 场景 B4：AprilTag Eye-to-Hand 手眼标定

**配置：** 相机固定桌面，AprilTag `DICT_APRILTAG_36h11` ID=0 刚性安装在机械臂末端。默认 Tag 边长 **6 mm**（`--tag-size 0.006`）；若实测为 60 mm 则用 `--tag-size 0.06`。

**前置：** 已完成 B3 内参标定；RoArm 与相机分别在 `RoArm_M3` / `Ubuntu-24.04` WSL。

**管理员 PowerShell（每次开机）：**

```powershell
powershell -ExecutionPolicy Bypass -File scripts/berxel/camera/attach_camera.ps1
powershell -ExecutionPolicy Bypass -File scripts/roarm/attach_usb.ps1
```

**终端 1 — RoArm_M3 驱动：**

```bash
wsl -d RoArm_M3
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/roarm/launch_driver.sh
```

**终端 2 — RoArm_M3 位姿服务：**

```bash
wsl -d RoArm_M3
bash .../scripts/roarm/launch_command_control.sh
ros2 service call /get_pose_cmd roarm_msgs/srv/GetPoseCmd
```

**终端 3 — Ubuntu-24.04 采集（移动机械臂，相机不动，S 保存，目标 15~20 组）：**

```bash
wsl -d Ubuntu-24.04
bash .../scripts/berxel/handeye/collect.sh --tag-size 0.006
```

**解算：**

```bash
wsl -d Ubuntu-24.04
bash .../scripts/berxel/handeye/solve.sh
```

输出：

| 文件 | 内容 |
|------|------|
| `calibration/hand_eye/extrinsics.yaml` | `T_base_cam` |
| `calibration/hand_eye/hand_eye_report.txt` | 残差 + `static_transform_publisher` 命令 |
| `calibration/hand_eye/samples/` | 样本图像与 JSON |

验收：样本 ≥ 15，平均残差平移 < 5 mm、旋转 < 1°。标定期间**不可移动相机**。

### 场景 B4-auto：全自动手眼标定

机械臂按预设路径循环运动，相机每 **1 秒**自动保存 AprilTag 样本，满 **25** 组后停臂并自动解算。

**推荐（快速版，与手动采集速度接近）：** 不读取 `joint_states`，仅用 TCP 位姿做约束（与手动 `collect.sh` 相同）。

```bash
wsl -d Ubuntu-24.04
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/handeye/auto_fast.sh \
  --tag-size 0.06 --target-samples 25 --fresh
```

**完整版（含 joint_states 抓夹监测，较慢）：**

```bash
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel/handeye/auto.sh \
  --tag-size 0.06 --target-samples 25 --fresh
```

**运动约束（标定专用）：**

| 约束 | 说明 |
|------|------|
| TCP `x ≥ 0`, `z ≥ 0` | 末端点在基座前方、地面以上 |
| 基础关节 `base_link_to_link1` | 相对零位 ±45°（fast 版由 TCP 估计） |
| 手腕关节2 `link4_to_link5`（roll） | 相对零位 ±45° |
| 抓夹关节 | `auto.sh` 监测 `link5_to_gripper_link`；`auto_fast.sh` 不监测 |

常用参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--tag-size` | `0.06` | Tag 外轮廓边长（米） |
| `--target-samples` | `25` | 有效样本数 |
| `--interval-s` | `1.0` | 采样间隔（秒） |
| `--fresh` | 关 | 备份旧 `samples/` 后重采 |
| `--dry-run` | 关 | 仅打印路径点，不运动 |
| `--no-preview` | 关 | 关闭 OpenCV 预览窗口 |

**本地验证（无需机械臂）：**

```bash
python3 scripts/berxel/handeye/waypoints_test.py
python3 scripts/berxel/handeye/auto.py --dry-run
python3 scripts/berxel/handeye/auto_fast.py --dry-run
python3 scripts/berxel/handeye/synthetic_test.py
```

| 脚本 | 作用 |
|------|------|
| `berxel/handeye/auto_fast.py` / `.sh` | **推荐** 快速自动采集 + solve（无 joint_states） |
| `berxel/handeye/auto.py` / `.sh` | 完整自动采集 + solve（含 joint_states 约束） |
| `berxel/handeye/waypoints.py` | 路径点与关节/TCP 约束 |
| `berxel/handeye/collect.py` / `.sh` | 跨 WSL 读 pose + AprilTag 手动采样 |
| `berxel/handeye/solve.py` / `.sh` | OpenCV `calibrateHandEye` 解算 |
| `berxel/handeye/common.py` | 公共变换 / AprilTag / RoArm 调用 |

---

## 维护约定

**新增脚本时，必须同步更新本 `README.md`：**

1. 在 [脚本速查表](#脚本速查表) 中增加一行。
2. 在 [按场景使用](#按场景使用) 中归入对应场景（或新建场景小节）。
3. 在 [各脚本详细说明](#各脚本详细说明) 中补充：用途、用法、依赖、注意事项。
4. 若改变推荐流程，更新 [推荐练习顺序](#推荐练习顺序)。

脚本文件头部建议保留一行注释，写明用途与运行环境，例如：

```bash
#!/usr/bin/env bash
# <一句话用途>（在 WSL 内运行；需 fake control 已启动）
```
