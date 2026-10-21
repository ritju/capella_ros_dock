---
feature: dock-pose-gate-and-buffer-collision
status: delivered
updated: 2026-08-11
branch: fix/dock-pose-gate-and-buffer-collision
commits: 66d46fc..6fcf483
---

# Dock Pose Gate and Buffer Collision

## Report

**What was built** — 在 `SimpleGoalController::get_velocity_for_position` 入口增加码系位姿一致性门控：当 `sees_dock` 为 true 时，用 `charger_pose_map * current_pose` 反推 map 系机器人位置，与 `robot_pose_map` 平面偏差超过 `params_ptr->max_translation` 则将本地 `sees_dock` 置 false，避免不可靠的码系位姿进入 `save_all_poses_infos` 及后续码系计算。同时移除 `MOVE_TO_BUFFER_POINT` 阶段靠近充电桩时跳过碰撞检查的忽视区域，该阶段在 `collision_check=true` 时始终做碰撞检查。

**Verification** — `colcon build --packages-select capella_ros_dock_msgs capella_ros_dock`（隔离 build/install 于 /tmp）PASS；独立评审确认两项验收标准均满足，tf2 乘法顺序正确，`GO_TO_GOAL_POSITION` 忽视逻辑未受影响。仓库无 `SimpleGoalController` 单测。

**Journey log**
- `current_pose` 是码系（charger→base_link_dummy）位姿，`robot_pose_map` / `charger_pose_map` 是 map 系；组合 `charger_pose_map * current_pose` 与既有 `charger_pose_map.inverse() * robot_pose_map` 约定一致
- `sees_dock` 是值参数，只改本地副本即可同时约束 `save_all_poses_infos` 与函数内后续分支，无需回写 `DockingBehavior::sees_dock_`
- 评审提示：`max_translation` 既是线速度上限又是本次位置偏差阈值，日后若为调速下调该值，门控会变敏感

## [S1] Problem

对接控制里两处可靠性缺口：

1. `get_velocity_for_position` 无条件信任 `current_pose`（码系机器人位姿）。当 `/pose_with_id_optimize` 不可靠时，后续基于码系的收敛判断、目标点计算会出错，而 `sees_dock` 仍为 true。
2. `MOVE_TO_BUFFER_POINT` 阶段存在「靠近充电桩且朝向桩运动则跳过碰撞检查」的忽视区域。该阶段机器人仍在长距离移动，不应跳过碰撞检查。

## [S2] Design

### S2.1 位姿一致性门控

在 `SimpleGoalController::get_velocity_for_position` 中，调用 `save_all_poses_infos` 之前：

1. 当 `sees_dock == true` 时，用码系位姿与桩在 map 下的位姿反推机器人 map 位姿：
   `robot_via_dock = charger_pose_map * current_pose`
2. 计算该位姿与 `robot_pose_map` 的平面位置偏差：
   `deviation = hypot(robot_via_dock.x - robot_pose_map.x, robot_via_dock.y - robot_pose_map.y)`
3. 若 `deviation > params_ptr->max_translation`（即 `max_translation * 1`），判定 `current_pose` 不可靠，将本地 `sees_dock` 置为 `false`，再进入 `save_all_poses_infos`。

要点：
- `sees_dock` 为值参数，只改本地副本；`DockingBehavior::sees_dock_`（话题可见性）不被修改。
- 只比较平面位置（x, y），不比较 yaw。
- 门控同时作用于 dock 与 undock 路径（二者共用本函数）。
- 偏差过大时打 DEBUG 日志，便于现场排查。

### S2.2 MOVE_TO_BUFFER_POINT 碰撞检查始终生效

在 `NavigateStates::MOVE_TO_BUFFER_POINT` 分支中：

- 移除「`yaw_c2r_abs < π/2 && x_c2r < dock_valid_obstacle_x` 则 `need_check_collision = false`」的忽视区域。
- 当 `params_ptr->collision_check` 为 true 时，始终执行碰撞检查。
- `GO_TO_GOAL_POSITION` 等其它阶段的既有忽视逻辑保持不变。

## [S3] Out of Scope

- 不改动 `/pose_with_id_optimize` 或上游 AprilTag 测量。
- 不调整 `max_translation` 阈值本身（沿用 `params_ptr->max_translation`）。
- 不改动 `GO_TO_GOAL_POSITION` 等其它阶段的碰撞忽视逻辑。
- 不新增单元测试框架（仓库无现成针对 `SimpleGoalController` 的单测）。

## Tasks

- [x] T1: 实现位姿一致性门控 — acceptance: `get_velocity_for_position` 在 `save_all_poses_infos` 前，当 sees_dock 且平面偏差 > `params_ptr->max_translation` 时将本地 sees_dock 置 false (covers: S2.1)
- [x] T2: 移除 MOVE_TO_BUFFER_POINT 碰撞忽视区域 — acceptance: 该阶段不再因靠近桩而跳过碰撞检查，`need_check_collision` 在 `collision_check=true` 时恒为 true (covers: S2.2; depends: T1)
