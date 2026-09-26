# Local patches to imported sources

`src/` is gitignored and `scripts/import_sources.sh` clones each dependency itself, so a
change made inside one of those checkouts is tracked nowhere and disappears the moment the
directory is removed and re-imported. Anything this cell depends on that is not upstream
lives here as a patch, with the upstream commit it applies to named in its own header.

`import_sources.sh` only clones when `.git` is absent, so re-running it does **not** clobber
an existing checkout — the risk is a deleted or freshly cloned `src/`, not a re-import.

## crisp_controllers-introspection.patch

Applies to `crisp_controllers` at upstream `0279dc8`, cloned depth-1 from the default branch
(`import_sources.sh:62`) — so the base is not pinned and this patch may need rebasing after
upstream moves.

`CartesianController` creates no publishers, so ros2_control introspection is the only way to
read what it is tracking. Upstream already exposes `tau_task`, `tau_desired`, `error`,
`target_*`, `q_target` and `q_ref` behind `enable_introspection`. The patch adds the rest of
the control law — `q_filtered`, `dq_filtered`, `dq_ref`, `tau_nullspace`, `tau_secondary`,
`tau_joint_limits`, `tau_wrench`, `tau_friction`, `tau_coriolis`, `tau_gravity`,
`desired_position_*`, `desired_orientation_*`, `target_wrench_*` — so that a learned
external-torque model can be ablated against each input separately rather than their sum.

All of them are existing members updated in place, so the cost is a pointer registration and
nothing in the realtime loop.

Verified on mock hardware: **126 channels** register and publish at **499.98 Hz**, the same
tick rate as `/lbr/lbr_state`, which is what lets the two streams be aligned tick-for-tick.

Introspection entries publish **only while the controller is active**, and registration runs
in `on_configure` — so `enable_introspection` must be set in `config/controllers.yaml` before
the controller is configured. Setting the parameter on an already-configured controller does
nothing.

To re-apply after a fresh import:

```bash
git -C src/crisp_controllers am < patches/crisp_controllers-introspection.patch
pixi run -e jazzy colcon build --base-paths . src --packages-select crisp_controllers \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## crisp_controllers-activate-clears-targets.patch

Applies on top of the introspection patch (upstream `0279dc8` + `fd64d0a` + `006d34a`).

**Safety.** The target subscriptions live from `on_configure`, and the `new_target_*` flags were
cleared only there. A target published while the controller was inactive therefore survived
activation and replaced the measured pose `on_activate` had just latched, on the first
`update()`. `crisp_session.py --home` runs a crisp_py `Robot`, which publishes its target pose and
joints at 50 Hz from the pose it STARTED at; arming the Cartesian controller after homing then
yanked the arm back to its pre-homing pose. On 25 Sep 2026 that hit FRI's velocity guard on A2
within milliseconds of activation and dropped the session with the camera 37 cm lower.

Both `CartesianController` and `CartesianAdmittanceController` now clear the pose, joint and
wrench flags and zero the wrench target in `on_activate`. A target that arrives after
activation is a real command and is applied as before.

Re-apply after a fresh import, after the introspection patch:

```bash
git -C src/crisp_controllers am < patches/crisp_controllers-activate-clears-targets.patch
pixi run -e jazzy colcon build --base-paths . src --packages-select crisp_controllers \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## crisp_controllers-rotational-stiffness-ceiling.patch

Applies on top of the activation patch.

`variable_max_stiffness.rotational` clamps `task.k_rot_*` (whose own bound is 5000) and was
validated to [0, 100]. On the iiwa14 with the UMI, rotation then trails its target by ~200 ms
whatever the damping: `scripts/track_tune.py` sweeps 2-3 (26 Sep 2026) fit
lag ~= (d_rot + ~11 hidden) / k_rot, the hidden part being below crisp. The ceiling is now 1000.
Nothing changes until a config or a `set_parameters` raises it; controllers_umi.yaml still
launches at 100.

Re-apply after a fresh import, after the activation patch:

```bash
git -C src/crisp_controllers am < patches/crisp_controllers-rotational-stiffness-ceiling.patch
pixi run -e jazzy colcon build --base-paths . src --packages-select crisp_controllers \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```
