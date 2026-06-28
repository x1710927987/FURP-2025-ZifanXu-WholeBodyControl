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

**Links (optional):** [`docs/moveit_pose_dataflow.md`](moveit_pose_dataflow.md), [`src/moveit_ee_tutorial/README.md`](../src/moveit_ee_tutorial/README.md), [`scripts/wsl_setup_moveit.sh`](../scripts/wsl_setup_moveit.sh)

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
