# Berxel iHawk100Q 标定数据

本目录存放 iHawk 100Q RGB 相机内参标定结果与原始采集图像。

## 文件说明

| 文件 | 说明 |
|------|------|
| `intrinsics.yaml` | 内参矩阵、畸变系数、RMS 重投影误差 |
| `calib_report.txt` | 每张标定图的角点检测状态 |
| `undistort_sample.jpg` | 去畸变前后对比样图 |
| `snapshot.jpg` | 相机连通性测试单帧截图 |
| `calib_images/` | 标定采集的棋盘格原图 |

## 标定参数

- 棋盘格：8×6 内角点，方格边长 25 mm
- 分辨率：640×400 MJPEG
- 相机：Berxel iHawk100（UVC `/dev/video0`）

## 重新标定

```bash
wsl -d Ubuntu-24.04
REPO=/mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl
bash "$REPO/scripts/berxel_calib_collect.sh" --auto
bash "$REPO/scripts/berxel_calib_run.sh"
```

脚本与详细说明见 [`scripts/README.md`](../scripts/README.md) 场景 B1–B3。

## 深度图 SDK

深度流仍使用 WSL 用户目录下的 Berxel SDK（`~/berxel_sdk`），与本目录 RGB 标定数据分开存放。
