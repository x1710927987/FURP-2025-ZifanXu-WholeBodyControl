# Weekly Progress Log

> Update this file **every week**. Add a new entry at the top for each week.
> This is the first thing we check during review. Keep it honest and specific — it also feeds your attendance record (Rule 1).

**How to use:** copy the *Week template* block below for each new week. Newest week goes at the top.

---

## Week template — copy me

### Week N — YYYY-MM-DD

**Attended this week's meeting:** Yes / No (if No, did you email leave? Yes / No)

**Progress this week**

- _What did you actually do / finish?_

**Challenges & blockers**

- _What got in the way? What are you stuck on?_

**Next steps**

- _What will you do next week?_

**Hours spent (optional):** _e.g. 6h_

**Links (optional):** _commits, notebooks, docs, datasets..._

---

<!-- =================  YOUR ENTRIES BELOW  ================= -->

### Week 4 — 2026-07-10

**Attended this week's meeting:** Yes

**Progress this week**

- Set up **RoArm-M3** on dedicated WSL distro `RoArm_M3` (Humble): USB attach via [`scripts/roarm/attach_usb.ps1`](../scripts/roarm/attach_usb.ps1), driver, RViz display, MoveIt2, Servo keyboard control, command-control services, and MTC demos (`cartesian` / `pick_place`)
- Completed **Berxel iHawk100Q RGB intrinsic calibration**: 25 checkerboard images (8×6, 25 mm), RMS reprojection **0.44 px** — see [`calibration/berxel/`](../calibration/berxel/) and scripts B1–B3 in [`scripts/README.md`](../scripts/README.md)
- Added Berxel camera USB attach, preview, snapshot, and OpenCV calibration scripts under [`scripts/berxel/`](../scripts/berxel/)
- Started **Eye-to-Hand hand-eye calibration** (AprilTag on arm, fixed camera): collected 20 pose–image pairs into [`calibration/hand_eye/samples/`](../calibration/hand_eye/samples/); solve step needs debugging (residuals far above target — likely Tag size mm vs m or frame convention)
- Verified RoArm TF tree (`base_link`, `hand_tcp`, `ihawk_color_optical_frame`) via `view_frames` (see `.local/Archive/frames_2026-07-10_15.14.29.pdf`)
- Pushed commit `4ba2db6` — Berxel calibration + RoArm-M3 WSL helpers

**Challenges & blockers**

- Hand-eye solve not yet passing acceptance (target: trans < 5 mm, rot < 1°); need to confirm physical AprilTag edge length (6 mm vs 60 mm) and `T_base_cam` convention
- RoArm and Berxel camera run in **two separate WSL distros** (`RoArm_M3` + `Ubuntu-24.04`); cross-distro pose sampling adds integration complexity
- Mechanical arm may move on USB connect — keep workspace clear

**Next steps**

- Fix hand-eye calibration (re-collect with correct `--tag-size`, re-run solve)
- Commit hand-eye scripts and document B4 workflow in `scripts/README.md`
- Run RoArm MTC `pick_place` end-to-end on hardware
- Pipe calibrated camera extrinsics into object pose → MoveIt grasp pipeline

**Hours spent (optional):** approximately 12h

**Links (optional):** [`calibration/berxel/README.md`](../calibration/berxel/README.md), [`calibration/hand_eye/README.md`](../calibration/hand_eye/README.md), [`scripts/README.md`](../scripts/README.md) (RoArm R1–R6, Berxel B1–B4), commit `4ba2db6`

### Week 3 — 2026-07-03

**Attended this week's meeting:** Yes

**Progress this week**

- Continued MOGI / ROS 2 communication learning: implemented **Service** client–server pattern in WSL clone `~/FURP-2025-ZifanXu-WholeBodyControl`
- Added interface package `bme_ros2_tutorials_interfaces` with `CustomCalc.srv` (`a`, `b` → `sum`)
- Wrote `service_server.py` and `service_client.py` under `src/ros2_learning/`; updated `package.xml`, `setup.py`, and publisher/subscriber launch files (local WSL workspace, not yet pushed to GitHub)
- Reviewed SEP Phase 1 checklist and ROS2 bootcamp material on **topic / service / action** selection and TF2 sensor frames (prep for camera + arm integration)
- Pushed Week 2 data-flow diagram to remote (`ebda48f` — [`docs/moveit_pose_dataflow.md`](moveit_pose_dataflow.md))

**Challenges & blockers**

- Service tutorial code lives only in WSL home clone; need to sync to `D:` repo and commit
- `colcon build` for new interfaces package requires rebuilding dependent `bme_ros2_tutorials_py`

**Next steps**

- Commit and push ROS 2 service tutorial code to GitHub
- Begin hardware setup: RoArm-M3 USB + Berxel camera via usbipd
- Complete camera intrinsic calibration before hand-eye work

**Hours spent (optional):** approximately 8h

**Links (optional):** [`docs/moveit_pose_dataflow.md`](moveit_pose_dataflow.md), WSL workspace `~/FURP-2025-ZifanXu-WholeBodyControl/src/ros2_learning/`, commit `ebda48f`

### Week 2 — 2026-06-26

**Attended this week's meeting:** Yes

**Progress this week**

- Set up `~/ros2_ws` with `panda_gz_moveit2` (jazzy) + `pymoveit2`; patched `mock_components` for Jazzy
- Verified joint-space and Cartesian end-effector motion via pymoveit2 on Panda fake control
- Added [`src/moveit_ee_tutorial/`](../src/moveit_ee_tutorial/) (README + `move_to_pose.py` wrapper)
- Added WSL helper scripts under [`scripts/`](../scripts/) (`wsl_launch_fake_control.sh`, `wsl_run_ee_tests.sh`, etc.)
- Verified collision-aware planning: `ex_collision_primitive.py` adds obstacle sphere; EE pose replanned
- Cloned MOGI Week 3-4 starter to `src/gazebo_basics/Week-3-4-Gazebo-basics` (local)
- Read `.local/hoppertrex_isaaclab` (mobile-base RL only; not EE planning) and SEP Phase 1 roadmap
- Drew MoveIt fake-control data-flow diagram for end-effector pose goals: nodes, ROS message/action types, and data directions — see [`docs/moveit_pose_dataflow.md`](moveit_pose_dataflow.md) (with CLI snapshots in [`docs/figures/`](figures/))

**Challenges & blockers**

- Jazzy spawner race: launch spawners often fail unless delayed respawn (`wsl_launch_fake_control.sh`)
- Gripper controller sometimes fails to configure; arm trajectory works
- Run `ros2 run` from **WSL terminal**, not PowerShell (quote truncation)

**Next steps**

- Complete MOGI Week 3-4 URDF/Gazebo exercises (clone starter package)
- Practice collision-aware planning (`ex_collision_primitive.py`)
- Re-test `ex_gz_control` + pose goals in Gazebo
- Obtain lab dual-arm URDF when available

**Hours spent (optional):** approximately 10h

**Links (optional):** [`docs/moveit_pose_dataflow.md`](moveit_pose_dataflow.md), [`src/moveit_ee_tutorial/README.md`](../src/moveit_ee_tutorial/README.md), [`scripts/wsl/setup_moveit.sh`](../scripts/wsl/setup_moveit.sh)

### Week 1 — 2026-06-22

**Attended this week's meeting:** Yes

**Progress this week**

- Relearnt the basic knowledges of ROS2 (including installation, topic, launch etc.)
- Typed all the codes from [MOGI-ROS/Week-1-2-Introduction-to-ROS2: Introduction to ROS2 Jazzy basics](https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS2/tree/main) manually (see ./src/ros2_learning)

**Challenges & blockers**

- All challenges are solved with the help of AI

**Next steps**

- Continue to learn other communication methods of ROS2

**Hours spent (optional):** approximately 6h

**Links (optional):** [MOGI-ROS/Week-1-2-Introduction-to-ROS2: Introduction to ROS2 Jazzy basics](https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS2/tree/main)

### Week 0 — 2026-06-15

**Attended this week's meeting:** Yes

**Progress this week**

- Set up repository from the FURP template.

**Challenges & blockers**

- None

**Next steps**

- Follow up

**Hours spent (optional): less than 30 min**

**Links (optional): [x1710927987/FURP-2025-ZifanXu-WholeBodyControl: The whole name of the project is Whole-Body Control for a Mobile Manipulator with QP + Reinforcement Learning](https://github.com/x1710927987/FURP-2025-ZifanXu-WholeBodyControl)**
