# Sim2Real Gap Analysis & Fix Plan

## Investigation Summary

The T1 robot vibrates (particularly ankles) when running the colosseum-trained policy via arena. A thorough comparison of three training+deploy stacks was performed:

| Stack | Training | Deploy | Parallel Mech Handling |
|---|---|---|---|
| **booster_gym** (original) | IsaacGym, heavy DR | Single-threaded Python | **YES** - kp=0 + direct torque |
| **booster_deploy** (next-gen) | N/A (deploy-only) | Multi-process Python | **NO** - standard PD |
| **holosoma** (reference) | IsaacSim, aggressive DR | ONNX inference | **NO** - standard PD |
| **colosseum + arena** (ours) | MuJoCo, light DR | C++ DDS | **NO** - standard PD |

### Key Insight

**booster_gym** is the ONLY stack with deployment-side parallel mechanism handling. Both **booster_deploy** and **holosoma** dropped it and instead rely on **aggressive domain randomization during training** to produce policies robust enough to handle the ankle nonlinearity without special handling. Since holosoma successfully trains and deploys without parallel mechanism handling, the root cause is **insufficient domain randomization in colosseum training**, not a missing deployment feature.

---

## Domain Randomization: What Works (booster_gym + holosoma Merged)

Both booster_gym and holosoma have proven sim2real transfer on T1-class humanoids. Here is a side-by-side merge of all their DR parameters, showing what colosseum already covers and what's missing.

### Complete DR Parameter Inventory

Source: `booster_gym/envs/t1.py` + `booster_gym/envs/T1.yaml`, `holosoma/.../config_values/loco/t1/randomization.py`

| # | DR Parameter | booster_gym | holosoma T1 | colosseum | Status |
|---|---|---|---|---|---|
| | **Actuator & Joint Dynamics** | | | | |
| 1 | Joint stiffness DR | `scale [0.95, 1.05]` | `scale [0.9, 1.1]` | **none** | 🔴 MISSING |
| 2 | Joint damping DR | `scale [0.95, 1.05]` | `scale [0.9, 1.1]` | **none** | 🔴 MISSING |
| 3 | Joint friction physics | `add [0, 2.0] Nm` | — | **none** | 🟡 MISSING |
| | **Mass & Inertia** | | | | |
| 4 | Base mass DR | `scale [0.8, 1.2]` | `add [-1.0, 3.0] kg` | **none** | 🔴 MISSING |
| 5 | Link mass DR | `scale [0.98, 1.02]` | `scale [0.9, 1.2]` | **none** | 🟡 MISSING |
| 6 | Base CoM offset | `add ±0.1 m` | ❌ disabled | `add ±0.025/0.03 m` | ✅ |
| | **Initial State** | | | | |
| 7 | Joint init noise | `add ±0.05 rad` | `scale [0.5, 1.5]` × | **0.0** | 🔴 MISSING |
| 8 | Base init pos xy | `add ±1.0 m` | — | `add ±0.5/0.5 m` | ✅ |
| 9 | Base init lin vel xy | `add ±0.1 m/s` | — | — | ⬜ |
| | **Contact & Friction** | | | | |
| 10 | Foot friction | `add [0.1, 2.0]` | `set [0.1, 1.0]` | `set [0.3, 1.2]` | ✅ |
| 11 | Ground compliance | `add [0.5, 1.5]` | — | **none** | 🟢 MISSING |
| 12 | Ground restitution | `add [0.1, 0.9]` | — | **none** | 🟢 MISSING |
| | **Perturbations** | | | | |
| 13 | Push force | `add [0, 10] N` | `max_vel [1.0, 1.0]` | `vel [0.5, 0.5, 0.4]` | ✅ |
| 14 | Push interval | `5.0 s` | `[5, 10] s` | `[1, 3] s` | ✅ |
| 15 | Push torque | `add [0, 2] Nm` | — | `ang [0.52, 0.52, 0.78]` | ✅ |
| 16 | Kick lin vel | `add [0, 0.1]` | — | — | ⬜ |
| 17 | Kick ang vel | `add [0, 0.02]` | — | — | ⬜ |
| | **Sensing & Timing** | | | | |
| 18 | Encoder bias | — | `±0.01 rad` (disabled) | `±0.015 rad` | ✅ |
| 19 | Action delay | — | `[0, 1] steps` | **none** | 🟢 MISSING |
| 20 | Observation noise | `ang_vel ±0.2, jpos ±0.01, jvel ±1.5` | `jpos ±0.01, jvel ±0.1` | `ang_vel ±0.2, grav ±0.05, jpos ±0.01, jvel ±1.5` | ✅ |

**Legend**: 🔴 CRITICAL (blocks sim2real), 🟡 HIGH (likely contributes to vibration), 🟢 MED (nice to have), ⬜ optional

### What colosseum already has (strong baseline)

```
✅ Foot friction              0.3–1.2
✅ Base CoM offset             ±0.025/0.025/0.03 m
✅ Encoder bias                ±0.015 rad
✅ Push robot                  every 1–3s, full velocity+torque
✅ Observation noise           ang_vel, gravity, joint_pos, joint_vel
✅ Base reset                  x/y/z/yaw
✅ Velocity curriculum         3 stages (0→1→2 m/s)
```

### What colosseum needs to add

**Must-add (present in BOTH booster_gym AND holosoma):**

1. **Joint stiffness DR** — scale kp per env by [0.9, 1.1] (holosoma) or [0.95, 1.05] (booster_gym). Pick either; holosoma's wider range is safer.
2. **Joint damping DR** — same scale range as stiffness.
3. **Joint init noise** — additive ±0.05 rad (booster_gym) or multiplicative scale [0.5, 1.5] (holosoma). Additive is simpler for mjlab's `reset_joints_by_offset`.
4. **Base mass DR** — scale [0.8, 1.2] (booster_gym) or add [-1.0, 3.0] kg (holosoma). Additive is easier for MuJoCo body mass modification.

**Should-add (present in ONE of the frameworks):**

5. **Joint friction physics** — booster_gym adds 0–2 Nm Coulomb friction per joint at episode reset. This directly addresses ankle vibration (2 Nm is 17% of ankle's 12 Nm effort limit). MuJoCo XML already has `frictionloss=0.1` on ankles — a startup event that scales this 1–20× covers it.
6. **Link mass DR** — booster_gym scales ±2%, holosoma scales [0.9, 1.2]. Holosoma's wider range is more aggressive; booster_gym's is more typical for locomotion.

**Nice-to-add (lower priority):**

7. **Ground compliance/restitution** — booster_gym only. Varies contact dynamics.
8. **Action delay** — holosoma only. One step delay trains policy to handle latency.

### How these gaps relate to ankle vibration

The T1 ankle has a parallel crank mechanism with only **12 Nm effort limit** (vs 45–60 Nm for hips/knees). Any sim2real mismatch is proportionally larger for ankles:

| Gap | How it causes ankle vibration |
|---|---|
| No kp DR | Real motor kp ≠ nominal → wrong effective ankle stiffness → oscillation |
| No kd DR | Real motor kd ≠ nominal → wrong damping → undershoot/overshoot cycles |
| No joint friction DR | Real joint has ~1-2 Nm static friction → policy trained without it overcorrects |
| No init noise | Policy never saw ankle not-at-default at start → brittle recovery |
| No mass DR | Wrong ankle loading → wrong ground reaction → policy fights itself |

---

## Deployment Fixes (arena)

### ✅ Fixed: Ankle damping 2.0 → 3.0

`src/tasks/T1VelocityFlat.cpp:188` — changed ankle damping from 2.0 to 3.0 Nm·s/rad to match colosseum training (`actuators.py:181`).

### Note: `parallel_joint_indices` is dead config

`T1VelocityFlat.cpp:205` sets `cfg.robot.parallel_joint_indices = {15, 16, 21, 22}` but no portal consumes this. Neither booster_deploy nor holosoma use deployment-side parallel mechanism handling (only the original booster_gym deploy did, and booster_gym's successor frameworks all dropped it in favor of DR during training). Keep the config field but don't implement deployment-side torque conversion — fix it in training instead.

---

## Training Fix Plan (colosseum `event_cfg.py`)

All changes go in `external/colosseum/src/colosseum/tasks/velocity/config/t1_23dof/event_cfg.py`.

### Fix 1: Joint Init Noise

```python
"reset_robot_joints": EventTermCfg(
    func=reset_joints_by_offset,
    mode="reset",
    params={
        "position_range": (-0.05, 0.05),    # was (0.0, 0.0)
        "velocity_range": (0.0, 0.0),
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
    },
),
```

### Fix 2: Actuator Stiffness/Damping DR

Requires a `startup` event that scales per-environment PD gains. Both frameworks use the same pattern — pick the wider `[0.9, 1.1]` range from holosoma to be safer:

```
kp[i] *= uniform(0.9, 1.1)
kd[i] *= uniform(0.9, 1.1)
```

Needs a new event function in mjlab (e.g., `actuator_gain_scale`) or check if one already exists.

### Fix 3: Base Mass DR

```python
"base_mass": EventTermCfg(
    mode="startup",
    func=body_mass_add,  # holosoma approach: add [-1.0, 3.0] kg
    params={
        "asset_cfg": SceneEntityCfg("robot", body_names=(BASE_BODY_NAME)),
        "range": (-1.0, 3.0),
    },
),
```

### Fix 4: Link Mass DR

```python
"link_mass": EventTermCfg(
    mode="startup",
    func=body_mass_scale,
    params={
        "asset_cfg": SceneEntityCfg("robot", body_names=(".*"), exclude=(BASE_BODY_NAME,)),
        "scale_range": (0.9, 1.2),  # holosoma range (wider but proven)
    },
),
```

### Fix 5: Joint Friction DR (booster_gym only — but highest ankle impact)

```python
"joint_friction": EventTermCfg(
    mode="startup",
    func=joint_frictionloss_scale,  # needs implementing in mjlab
    params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
        "scale_range": (1.0, 20.0),  # ankle frictionloss 0.1 → 0.1-2.0 Nm
    },
),
```

### Fix 6: Ground Compliance + Restitution (booster_gym only)

```python
"ground_compliance": EventTermCfg(
    mode="startup",
    func=geom_solref_solimp,  # needs implementing
    params={
        "asset_cfg": SceneEntityCfg("terrain"),
        "compliance_range": (0.5, 1.5),
        "restitution_range": (0.1, 0.9),
    },
),
```

### Fix 7: Action Delay (holosoma only)

```python
"action_delay": EventTermCfg(
    mode="startup",
    func=action_delay_buffer,  # needs implementing
    params={
        "delay_range": (0, 1),  # steps
    },
),
```

### Implementation Priority

```
Must fix before next hardware test:
  1. Joint stiffness DR (kp scale)
  2. Joint damping DR (kd scale)
  3. Joint init noise (±0.05)
  4. Base mass DR

Should fix (high ankle impact):
  5. Joint friction DR (booster_gym's 0-2 Nm)
  6. Link mass DR

Can defer (lower impact):
  7. Ground compliance/restitution
  8. Action delay

---

## Additional Considerations

### Training Duration

holosoma uses per-env randomization (each of 4096 envs gets different DR params). Colosseum should match this pattern — randomization should be per-environment, not per-episode, so the policy sees the full distribution continuously.

### Physics Timestep

- colosseum: dt=0.005, decimation=4 (50 Hz control)
- booster_gym: dt=0.002, decimation=10 (50 Hz control)
- holosoma: dt=0.002, decimation=10 (50 Hz control)

The timestep difference (0.005 vs 0.002) may affect contact dynamics. Consider reducing colosseum's timestep or verifying convergence at 0.005.

### Episodic vs Persistent DR

- booster_gym: Resets DR params at episode boundaries
- holosoma: DR params fixed at env creation (persistent across episodes)

The persistent approach (holosoma) is likely better — the policy encounters consistent but different dynamics per env, encouraging robust adaptation.

---

## Verification Plan

After retraining with full DR:
1. **Sim2sim**: Deploy retrained policy in arena's MuJoCo sim (`--backend mujoco`) with perturbed physics
2. **Sim2real**: Deploy on hardware once sim2sim shows stable walking
3. **Metrics to track**: ankle oscillation amplitude (std of joint_vel at ankles > 5 rad/s → vibration), foot slip frequency, tracking error
