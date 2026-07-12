# Eye-to-Hand 手眼标定数据（AprilTag）

## 配置

| 项目 | 值 |
|------|-----|
| 类型 | Eye-to-hand（相机固定桌面，Tag 在机械臂末端） |
| Tag | `DICT_APRILTAG_36h11`, ID=0 |
| 默认 Tag 边长 | **6 mm**（0.006 m）——请用尺子复核；若实为 60 mm 则采集时加 `--tag-size 0.06` |
| RGB 内参 | [`../berxel/intrinsics.yaml`](../berxel/intrinsics.yaml) |
| 机器人 TCP | `hand_tcp`，基座 `base_link` |

## 目录

| 路径 | 说明 |
|------|------|
| `samples/` | 采集样本（`sample_NNN.jpg` + `sample_NNN.json`） |
| `extrinsics.yaml` | 求解结果 `T_base_cam` |
| `hand_eye_report.txt` | 残差与 static_transform_publisher 命令 |

## 流程

```bash
# 1. 管理员 PowerShell：attach 相机 + 机械臂 USB
# 2. RoArm_M3 终端1：roarm_launch_driver.sh
# 3. RoArm_M3 终端2：roarm_launch_command_control.sh
# 4. Ubuntu-24.04：
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel_handeye_collect.sh --tag-size 0.006
bash /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/scripts/berxel_handeye_solve.sh
```

若 Tag 实测为 60 mm：

```bash
bash .../berxel_handeye_collect.sh --tag-size 0.06
bash .../berxel_handeye_solve.sh --tag-size 0.06
```

## 验收

- 有效样本 ≥ 15
- 平均残差：平移 < 5 mm，旋转 < 1°
- 标定期间相机位置不可改变
