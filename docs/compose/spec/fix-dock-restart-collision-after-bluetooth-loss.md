---
feature: fix-dock-restart-collision-after-bluetooth-loss
status: delivered
updated: 2026-06-09
branch: fix/dock-restart-collision-after-bluetooth-loss
commits: 2d75a93..94f66af
---

# Fix dock restart collision after GO_TO_GOAL_POSITION bluetooth loss

## Report

**What was built** — Restarting dock after `GO_TO_GOAL_POSITION` stops on Bluetooth loss no longer drives into the charger. Near-dock restart (`|x_charger| <= 0.75m`) skips the buffer-point detour and goes straight to `ANGLE_TO_GOAL`. `drive_back` polarity and the buffer-point yaw transform are corrected so far starts and near-dock restarts both move toward the buffer with the right velocity sign. `GO_TO_GOAL_POSITION` drops already-passed intermediate goals without translational velocity instead of rapidly popping them while still driving. Collision-check exemption shrinks from 1.2m to contact range (0.5m). `initialize_goal`/`reset` fully clear per-run state. Dock/undock goal overlap is rejected, cancel releases the running flag immediately, and `cleanup_func` finalizes only the replaced goal so cancel→fast-restart cannot orphan a CANCELING/EXECUTING goal or clobber the successor.

**Verification** — `colcon build --merge-install --base-paths ./capella_ros_dock_msgs ./capella_ros_dock` (ROS 2 humble + workspace install sourced): PASS (2 packages). No unit suite exists for `SimpleGoalController` (only manual `test_dock` harness). Independent review accepted T1–T5 first pass; T6 required three iterations (cancel-race → orphaned CANCELING goal → `cleanup_func` clobber of successor) and was accepted on the final check.

**Journey log**
1. `drive_back` looked inverted, but only fails when the robot is *closer* to the dock than the buffer point — normal far starts hid the bug via a double-negative from the broken yaw=0 buffer transform.
2. The buffer/goal points target the aruco dummy frame; applying `dummy_dis·(cos yaw, sin yaw)` to both endpoints cancels the offset and yields the correct dummy-to-buffer vector.
3. ROS 2 Humble `handle_cancel` runs *before* `_cancel_goal()`, so `is_canceling()` is false inside the cancel callback — finalize from `execute_*` or `cleanup_func` only.
4. `BehaviorsScheduler::set_behavior` runs old `cleanup_func` *after* `handle_accepted` already set running flags / `initialize_goal`; cleanup must not touch shared state.
5. Residual: `dock_valid_obstacle_x=0.5` needs one real docking run to confirm the 0.5–1.2m band does not early-stop on dock costmap geometry (`yaw_c2r_abs < π/2` gate depends on external `/charger/pose`).

## [S1] Problem

After `GO_TO_GOAL_POSITION` low-speed phase stops the robot due to Bluetooth loss, restarting dock causes a collision. The robot is left near the charger (`|x| < 0.6m`); the restart path mis-judges motion direction / goal convergence and drives into the dock, with collision checking already disabled nearby.

## [S2] Design

### Root causes addressed

1. **Near-dock restart takes the buffer path with wrong `drive_back`**
   - `LOOKUP_MARKER` three-condition requires `|x| > 0.75m` to enter `ANGLE_TO_GOAL`. Near-dock restart fails it and goes to `ANGLE_TO_BUFFER_POINT` / `MOVE_TO_BUFFER_POINT`.
   - `drive_back` true/false is inverted relative to `theta_positive` / `theta_negative` (reverse-heading vs forward-heading angular error).
   - Buffer target `x_base_link = buffer_x - base_link_dummy_dis` assumes yaw=0, while robot `base_link` is yaw-rotated → wrong distance/angle when `yaw≈π`.

2. **`GO_TO_GOAL_POSITION` false convergence while still moving**
   - `|robot.x| < |gp.x|` treats "already past goal" as converged but still commands `linear.x`, so a restart past intermediate goals rapidly pops the queue while driving toward the dock.

3. **Collision check fully disabled within `dock_valid_obstacle_x` (1.2m)**
   - Entire near-dock restart path sits inside the exemption radius, so wrong velocity is not blocked.

4. **Incomplete reset on re-init**
   - `initialize_goal` / `reset` do not clear `pose_x_init_*`, `drive_back`, contact/camera flags.

5. **Dock goal re-entry not rejected while a behavior is running**
   - `handle_dock_goal` guard is commented out; cancel left `running_dock_action_` set until the next control tick.

### Contracts

- When robot is already inside the second-goal zone (`|x_charger| <= distance_tmp + deviate_second_goal_x`), `LOOKUP_MARKER` / `ANGLE_TO_X_POSITIVE_ORIENTATION` enter `ANGLE_TO_GOAL` directly (no buffer travel).
- `drive_back == true` iff reverse-heading angular error is the smaller one; `dist_buffer_point_yaw` is the matching error term.
- Buffer target is converted with the same yaw convention as robot `base_link`.
- Goals already behind the robot are skipped without translational velocity.
- `initialize_goal` clears per-run state listed above.
- New dock goal is rejected while another dock/undock behavior is active; cancel releases the running flag immediately; `cleanup_func` finalizes only the replaced goal handle.

## [S3] Out of Scope

- Charge-manager session / bluetooth stack changes.
- Costmap or footprint changes.
- Changing `bluetooth_lost_report_delay` semantics (report-only remains).

## Tasks
- [x] T1: LOOKUP_MARKER / ANGLE_TO_X skip buffer when already near dock — acceptance: near-dock restart does not command motion toward buffer behind the robot (covers: S2)
- [x] T2: Fix drive_back polarity and buffer point yaw transform — acceptance: far start and near-dock restart both move toward buffer with correct velocity sign (covers: S2)
- [x] T3: GO_TO_GOAL_POSITION skip passed goals without linear.x — acceptance: restart past intermediate goals does not rapidly drive into dock (covers: S2)
- [x] T4: Tighten collision-check exemption near dock — acceptance: exemption only applies at contact-range distance, not 1.2m (covers: S2)
- [x] T5: Full per-run reset in initialize_goal — acceptance: second dock attempt does not reuse pose_x_init_/drive_back (covers: S2)
- [x] T6: Reject overlapping dock goals — acceptance: second dock goal while behavior active is rejected; cancel→restart not blocked (covers: S2)

