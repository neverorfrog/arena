# Arena Robot Deployment — Synthesis

## 1. Initial Problem: Crash Loop

**Symptom**: Service SEGFAULT, restart counter climbing, DDS participant creation failed.

**Root cause**: `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/fastdds_profile.xml` pointed to a file that didn't exist on the robot.

**Fix**: Changed to `/opt/booster/BoosterRos2/fastdds_profile.xml` (matching the path used in the `maximus` service source and the `spqrbooster2026` repo).

---

## 2. Robot Kicking Around on Activation

**Root cause**: The activation sequence was broken.

**Old arena flow**:
```
changeMode(kWalking)          ← firmware's own walking controller takes over
prepare()                     ← sends DDS commands… but firmware ignores them in kWalking mode!
changeMode(kCustom)           ← abrupt switch, robot snaps to first policy target
policy loop
```

The `prepare()` commands were silently discarded because the firmware only reads `rt/joint_ctrl` in kCustom mode. The robot never moved to the prepare pose. When kCustom activated, the first policy output targeted the prepare pose from wherever the robot actually was → huge position error → huge torques → kicking.

**New arena flow** (current, matches booster_deploy sequence):
```
burst-hold at current pos × 100ms at 500 Hz (prepare kp/kd)
changeMode(kCustom)           ← firmware starts reading our DDS buffer
smoothPrepare()               ← interpolate current → prepare pose over 1s at 500 Hz
policy loop
```

The 100ms hold burst fills the DDS buffer with safe targets before the mode switch, and `smoothPrepare` ramps to the prepare pose after kCustom activates.

---

## 3. Current Issue: Leg Vibration After Policy Start

**Symptom**: Robot stands but legs vibrate with "incredible force." Less severe with reduced prepare gains, but still present.

**Observation from diag logs**:
- Ankle velocities spike to 2-5 rad/s within seconds of activation
- pg z-component oscillates but stays > -0.84 (robot tilts but recovers)
- Policy outputs very large ankle actions (last_action: LAP = 1.0-2.0 in raw output, which translates to 0.25-0.50 rad targets with action_scale=0.25)
- The `vel_command` is `[0,0,0]` — zero velocity, robot should just stand

**Prepare gains** (what we send during the 1s `smoothPrepare`):
```
Current arena (reduced):
  Ankle: kp=100, kd=3.0
  Hips:  kp=200, kd=5.0
  Duration: 1.0s

booster_deploy (original):
  Ankle: kp=450, kd=0.5      ← we tried this, caused even worse vibration
  Hips:  kp=350, kd=7.5
  Duration: 1.0s (500 steps × 2ms)
```

**Running gains** (after prepare, same in both arena and booster_deploy):
```
  Ankle: kp=50,  kd=3.0
  Hips:  kp=200, kd=5.0
```

---

## 4. Booster_deploy vs Arena — Structural Differences

| | booster_deploy | arena (current) |
|---|---|---|
| **Activation flow** | hold → kCustom → smooth interp 1s | hold burst → kCustom → smoothPrepare 1s |
| **Publish rate during prepare** | 500 Hz with EMA | 500 Hz (no EMA) |
| **Publish rate during policy** | 500 Hz with EMA (dedicated thread) | **50 Hz** (main loop) |
| **CmdType** | SERIAL | SERIAL |
| **Ankle control (running)** | **Torque mode**: `kp=0, tau = (target-actual) * stiffness` | **Position PD**: `q=target, kp=50, kd=3` |
| **Target filtering** | `filtered = filtered*0.8 + target*0.2` at 500 Hz | **None** |
| **Firmware** | `Gait` binary, `use_pos2torq_convert_=true` | Same `Gait` binary |

---

## 5. The EMA Filter in booster_deploy — Explained

From `booster_gym/deploy/deploy.py` line 176:
```python
# Runs at 500 Hz in a dedicated thread (_publish_cmd)
self.filtered_dof_target = self.filtered_dof_target * 0.8 + self.dof_target * 0.2
```

The policy runs at 50 Hz. When it outputs a new target (say 0.5 rad), the EMA ramps smoothly between the old and new value over ~10 iterations at 500 Hz:

```
t=0ms:  filtered = old*0.8 + 0.5*0.2
t=2ms:  filtered = (old*0.8+0.1)*0.8 + 0.5*0.2
...
t=20ms: filtered ≈ 89% of new target (next policy update arrives)
```

**Purpose**: Smooth the trajectory between 50 Hz policy updates so the target doesn't jump discontinuously at the firmware. At 50 Hz with PVT already enabled, the firmware does its own interpolation — the EMA is partially redundant. The structural benefit is the 500 Hz publishing rate, not the filter itself.

---

## 6. Ankle Torque Mode — The Biggest Difference

booster_deploy's publish loop (lines 183-190):
```python
# For ankle joints (indices 15, 16, 21, 22):
motor_cmd[i].q = self.dof_pos_latest[i]                          # freeze at actual position
motor_cmd[i].tau = (filtered_dof_target[i] - dof_pos_latest[i]) * stiffness[i]  # compute torque
motor_cmd[i].kp = 0.0                                             # disable onboard PD
```

**What this does**: The Python side computes `torque = position_error * stiffness` and sends raw torque commands with kp=0, kd=0. The firmware sees kp=0 and passes the torque through directly to the motor, bypassing its own PD controller and the `use_pos2torq_convert_` parallel mechanism conversion.

**What arena does**: Sends `q=target, kp=50, kd=3` for all joints including ankles. The firmware's `parallel_mech_input_custom_mode` with `use_pos2torq_convert_=true` internally converts serial ankle targets to parallel motor targets, then applies its own PD (or possibly torque conversion) with unknown internal gains.

**Why this matters**: If the firmware's internal ankle conversion uses different effective stiffness than arena's kp=50, the ankle response will differ from what the policy expects.

---

## 7. Training Frequencies

Both training systems run the policy at **50 Hz**:

| | Physics | Decimation | Policy |
|---|---|---|---|
| Colosseum (arena's model) | 200 Hz (dt=0.005) | 4 | **50 Hz** |
| booster_gym | 500 Hz (dt=0.002) | 10 | **50 Hz** |

The physics rate difference (200 vs 500 Hz) doesn't matter for deployment. The policy was trained with actions held constant for all substeps between policy calls in both cases.

Running the policy at a different frequency or with an EMA filter changes the effective dynamics the policy sees — the policy wasn't trained for it and may perform worse.

---

## 8. Sim vs Real booster-motion

| | Sim binary | Real binary |
|---|---|---|
| Executable | `booster-motion -mode sim` | `Gait` |
| Config file | `common_module_options_t1_rl_isaac.lua` | `common_module_options.lua` |
| `use_pos2torq_convert_` | **absent** | **true** |
| `torque_limit_` | absent | **50 Nm** |
| `action_hub` section | absent | present (WBC hand planner) |
| `head_kp_rl_mode` | absent | **{10, 10}** (overrides head kp) |
| DDS command handling | Always active (accepts commands in any mode) | Mode-gated (kCustom only) |

**Why sim works but real didn't**: The sim accepts DDS commands in any mode and has no `use_pos2torq_convert_`, so the old prepare-before-kCustom code happened to work. The real Gait binary is mode-gated and has the ankle parallel mechanism conversion.

---

## 9. Action Scaling and Joint Ordering

**Verified correct**:
- `action_scale = 0.25` uniform matches Colosseum training exactly (`ACTION_SCALE = {name: 0.25 for name in JOINT_NAMES}`)
- Joint ordering: identity mapping (arena, SDK, Colosseum training all use same order)
- `default_joint_pos` matches Colosseum `HOME_QPOS`
- `kp_factor/kd_factor = 1.0` in firmware config (DDS kp/kd pass through unscaled)

---

## 10. Remaining Issues / Open Questions

1. **Vibration root cause**: The policy-mechanical loop is unstable. Policy outputs large ankle actions → mechanical response → policy sees oscillating velocity → larger corrective actions → oscillation amplifies. The 50 Hz sampling aliases the mechanical resonance frequency.

2. **Possible fixes to try**:
   - Increase ankle kd from 3 → 5 or 6 (more mechanical damping)
   - Increase hip kd from 5 → 8
   - Send ankle commands in torque mode (kp=0, compute tau server-side like booster_deploy)
   - Add a 500 Hz publish thread + EMA filter (matched to booster_deploy)
   - Low-pass filter joint velocities in the observation

3. **Ankle torque mode**: Whether to implement booster_deploy's ankle torque mode in arena `publishCommand`. This is the biggest structural difference and would require reading the current joint position at publish time to compute `tau = (target - actual) * kp`. But arena's state arrives asynchronously from DDS, so the `actual` position might be stale by a few ms.

4. **Gains**: Whether to match booster_deploy's prepare gains exactly (kp=450/350) or keep the reduced values. The higher gains caused stronger vibration but that might be mitigated by other fixes (torque mode, target filtering).

---

## 11. Files Modified

| File | Change |
|---|---|
| `scripts/arena.service` | DDS profile path: `/opt/booster/` → `/opt/booster/BoosterRos2/` |
| `scripts/arena_start.service` | Same |
| `scripts/arena_stop.service` | Same |
| `src/portals/RobotPortal.cpp` | `CmdType::PARALLEL` → `SERIAL`; `prepare()` → `smoothPrepare()` with 500 Hz + interpolation; diagnostic logging |
| `include/portals/RobotPortal.h` | `prepare()` → `smoothPrepare()`; added `debug_counter_` |
| `src/main.cpp` | Full rewrite: removed joystick combos, kWalking mode, old activation; added hold burst + kCustom-first activation + smoothPrepare + EMA target filter (0.7/0.3) |
| `src/tasks/T1VelocityFlat.cpp` | Prepare gains changed from running-level to standalone values; ankle kp=100/kd=3, hip kp=200/kd=5; duration 1.0s |
| `external/simbridge/.../common_module_options_t1_rl_isaac.lua` | Added `use_pos2torq_convert_=true`, `serial_vel_filter_weight_=1.0`, `torque_limit_=50.` to sim config (for testing, later removed) |
