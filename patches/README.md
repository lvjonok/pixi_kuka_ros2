# Patches to crisp_controllers

`scripts/import_sources.sh` clones `crisp_controllers` at `0279dc8` (the pin is in
`repos/repos-fri-1.17.yaml`) and puts these four diffs on it with `git apply`, in the order
below. Each one applies on the one before it. A checkout that already has them is left alone.

They change `CartesianController` and `CartesianAdmittanceController`.

## 1. crisp_controllers-introspection.patch

The controller publishes nothing, so ros2_control introspection is how its terms are read.
Upstream registers `tau_task`, `tau_desired`, `error`, `target_*`, `q_target` and `q_ref` behind
`enable_introspection`.

- The patch registers the rest of the control law: `q_filtered`, `dq_filtered`, `dq_ref`,
  `tau_nullspace`, `tau_secondary`, `tau_joint_limits`, `tau_wrench`, `tau_friction`,
  `tau_coriolis`, `tau_gravity`, `desired_position_*`, `desired_orientation_*` and
  `target_wrench_*`. They are members the controller already updates, so the realtime loop does
  no more work.
- Upstream registers `tau_desired` and `tau_joint_limits` by the address of an Eigen vector that
  is later assigned by value, which frees the buffer the address points into: those two
  channels then publish freed memory. The patch copies both into arrays that stay in place and
  registers those.

Registration runs in `on_configure` and the entries publish only while the controller is
active, so `enable_introspection` is set in `config/controllers.yaml`; setting it on a
configured controller does nothing.

## 2. crisp_controllers-activate-clears-targets.patch

Safety. The target subscriptions live from `on_configure`, and upstream clears the
new-target flags only there. A pose, joint or wrench target published while the controller is
inactive then survives activation and replaces the measured pose `on_activate` has just
latched: on the first update the arm is driven to a stale target. A crisp_py `Robot` publishes
its target at 50 Hz from the pose it started at, so any client that stays connected across a
joint move leaves such a target behind.

The patch clears the pose, joint and wrench flags and zeroes the wrench target in
`on_activate`. A target that arrives after activation is applied as before.

## 3. crisp_controllers-rotational-stiffness-ceiling.patch

`variable_max_stiffness.rotational` clamps `task.k_rot_*`, and upstream validates it to
[0, 100] Nm/rad. The patch raises the bound to 1000. `config/controllers_umi.yaml` sets 250,
outside upstream's bound.

## 4. crisp_controllers-activate-zeroes-torque.patch

Safety. `tau_previous` seeds the torque-rate limit and the output filter, and upstream zeroes
it only in `on_configure`, so an activation starts from the torque the last one ended on and
the filter replays it for tens of milliseconds: the arm moves at the moment it is armed. The
patch zeroes it in `on_activate`. Zero is the neutral overlay, because Sunrise compensates
gravity under the FRI torque overlay.
