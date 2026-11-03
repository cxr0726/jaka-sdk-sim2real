# Khan SuperMimic ZMQ deployment

Run from `sim/demo`:

```bash
./scripts/run_demo.sh --viewer --duration 0 --mode jakaminimimic_superzmq
```

The independent `jakaminimimic_superzmq` section of `config/KpiConfig.yaml`
selects its ONNX through `policy_path` and the existing remote Pico
endpoint. `--direct-fsm jakaminimimic_superzmq` also works. The controller
waits for enabled joints before entering the new FSM; it does not reuse the
START/BACK combinations assigned to the older modes.

For local NPZ replay, the dedicated launcher starts control first, waits for a
valid state from that process, then starts the publisher at frame zero:

```bash
./scripts/run_super_npz_demo.sh scripts/newsmallstep.npz
./scripts/run_super_npz_demo.sh /path/to/training/walk1_subject1.npz
# Bounded run without a viewer:
KPI_SUPER_NPZ_VIEWER=0 ./scripts/run_super_npz_demo.sh scripts/newsmallstep.npz --duration 15
```

The publisher preserves all 27 joints and automatically distinguishes legacy
Isaac NPZ arrays from named Khan training NPZ arrays. Training joint/body
names determine the mapping; their raw waist index is not assumed to be 13.
Both formats are converted to JAKA wire order with the base reference in slot 0. Playback follows NPZ fps and interpolates to 50 Hz. After the last
frame the publisher continues static heartbeats; closing the viewer or Ctrl+C
cleans up this run's processes.

Replay uses a generated local config, motion port 28721 and state port 28731.
Logs are under `build/super_npz_replay/`. Source YAML and the remote endpoint
are not rewritten. The default height offset is 0.05 m for legacy Pico files
and 0 for named training files; `--height-offset` overrides it. Other optional
controls are `--npz-layout auto|isaac|jaka`, `KPI_SUPER_NPZ_MOTION_PORT`,
`KPI_SUPER_NPZ_STATE_PORT`, `KPI_SUPER_NPZ_RUN_DIR`,
`KPI_SUPER_NPZ_READY_TIMEOUT` and `KPI_SUPER_NPZ_VIEWER`.

`FSMMimicJakaMiniSuperZmq.cpp` follows the existing ZMQ FSM's state acquisition,
motion-window sampling, yaw alignment, observation construction, history,
inference and trajectory structure. It uses the shared realtime buffer and
the same source-clock protocol. Startup and each new ready generation rebuild
yaw alignment and history. Normal control ticks consume one atomic window.

## Model contract and joint order

The new actor has one float32 input `obs[batch,620]` and one float32 output
`actions[batch,27]`. Its dynamic batch dimension is resolved to one for this
controller. `SuperMimicDeployment` reads the ONNX's embedded metadata directly;
the adjacent export JSON is a report, not a runtime dependency.

Three orders are kept explicit:

- Wire/reference: existing JAKA order, left leg, right leg, waist, left arm,
  right arm, neck. All angles describe physical joints, including ankle PR.
- Shared buffer: existing Isaac order, retained for the older policies.
- Policy: the model's `joint_names` metadata, matched by name to both the
  deployment joints and the shared buffer. The supplied model uses JAKA order.

For the supplied model:

```text
policy_to_deploy =
0,1,2,3,4,5,12,13,14,15,16,17,24,6,7,8,9,10,11,18,19,20,21,22,23,25,26

policy_from_buffer =
0,3,8,13,17,21,1,4,9,14,18,22,2,5,10,15,19,23,25,7,12,16,20,24,26,6,11
```

The first map gathers measured angles/velocities and scatters output targets.
The second gathers every reference frame's joint angles. Previous actions stay
in policy order throughout. Deployment joint names are validated against the
SDK's fixed legacy layout; missing, duplicate or incompatible names fail at
startup. A legacy ONNX with the same tensor dimensions is rejected by schema
validation.

## Observation and action semantics

| Input slice | Contents |
|---|---|
| `[0:15]` | Five reference displacements in the first reference's full local rotation |
| `[15:20]` | Five reference world heights |
| `[20:155]` | Five absolute joint references in policy order |
| `[155:185]` | Five future reference rotations relative to current actual base |
| `[185:200]` | Five projected-gravity history samples |
| `[200:215]` | Five base angular-velocity history samples, multiplied by 0.5 |
| `[215:350]` | Five measured `q - q0` history samples in policy order |
| `[350:485]` | Five joint-velocity history samples, multiplied by 0.2 |
| `[485:620]` | Five previous raw action history samples |

History is oldest to newest within each term. Reset repeats the first sensor
sample five times with previous action zero. The future orientation window is
recomputed on every tick and is not itself stacked into history. Rot6d is
`R00,R01,R10,R11,R20,R21`. No running mean/variance normalization is added.

Both the reference and measured IMU anchor are `base_link`. The input wire's
base is body index 0. The policy does not use the waist quaternion or add the
waist joint's angular velocity. Existing SDK sensor-axis compensation and
parallel-ankle state conversion are retained.

### Reference body selection in the older modes

SuperMimic always uses base_link at wire index 0, as required by its model.
Its replay launcher does not provide an anchor selector.

Only the non-SuperMimic modes expose the buffer constructor's
`anchor_body_index` through YAML:

```yaml
jakaminimimic_zmq:
  reference_body_index: 0   # JAKA wire: base_link
  # reference_body_index: 13  # JAKA wire: waist_yaw_Link

jakaminimimic_zmq_fixwrist:
  reference_body_index: 0
```

`GlobalVars.cpp` loads this key and the respective FSM passes it to
`RealtimeMotionBuffer`. Values must be integers in 0..4095, and incoming
packets must contain the selected body slot. You do not need to modify
`RealtimeMotionBuffer.cpp` to select 13.

This selector changes the incoming reference pose. For an older model that
was trained with a waist anchor, also verify the measured IMU/orientation and
angular-velocity convention; changing the reference index alone does not
change those measurements. Confirm the sender uses the expected JAKA body
order: some record replay tools copy one selected pose into every body slot,
so their slot index does not distinguish base from waist.

The model metadata supplies `q0`, per-joint action scales and PD gains.
Selected SuperMimic runs map these values into SDK order before constructing
the backend. The action rule is:

```text
target_deploy[policy_to_deploy[p]] = q0[p] + action_scale[p] * raw_action[p]
```

The supplied action scales are all 0.5. Output actions and PD targets are not
clipped, smoothed or passed through an additional tanh. The original physical
model's effort/joint constraints remain in the SDK. Nonfinite actions/targets
stop the controller. Neck actions are included in all 27 outputs.

## Timing and height configuration

### Parameter precedence: ONNX metadata and YAML

For SuperMimic, startup reads YAML first, loads the ONNX embedded metadata in
`SuperMimicDeployment`, then `KpiSimMain.cpp` maps the model's parameters by joint
name into SDK order **before constructing the backend**. The latter values win:

| Embedded ONNX field | Runtime destination | YAML value superseded in SuperMimic |
| --- | --- | --- |
| `default_joint_pos` | `mDefaultPos`, `mInitTarget` | `default_angles`, `init_target` |
| `joint_stiffness` | `mKp`, then four `mKp_PR` entries | `kp`, `kp_pitch_roll` |
| `joint_damping` | `mKd`, then four `mKd_PR` entries | `kd`, `kd_pitch_roll` |
| `action_scale` | FSM's per-policy-joint target scaling | generic global `action_scale` is not used by this FSM |

This happens once at startup. It makes INIT, SDK gains, policy observations and
policy targets share the actor's training definitions; it does not add a second
PD controller. With `angle_map=0`, `FSMInit` zeros the four ankle MIT gains and
uses the dedicated pitch/roll PD path. The ankle SDK indices are `[4,5,16,17]`.
Policy indices differ: e.g. right hip pitch `6 -> 12`, left shoulder roll
`14 -> 7`. An array copy without this name mapping would assign parameters to
the wrong joints.

For the current `khan_lafan1_delay_covex` new 100k ONNX, the model's defaults,
initial targets and PD gains equal the corresponding current YAML values after
`KpiConfig.sim.yaml` is merged (all maximum absolute differences zero). Final PR
Kp is `[100,50,100,50]`, Kd `[2,.5,2,.5]`. Nevertheless, editing only these YAML
fields will not override the model in this mode. Older modes do not execute the
SuperMimic startup override.

`joint_damping` here means actuator **PD Kd**, not XML passive joint damping.
This code does not apply the metadata's armature, body mass/inertia or contact
geometry to the XML. Runtime settings such as ZMQ address, buffer clock, reference
height offset, initial base pose and XML selection remain YAML/CLI settings.
The adjacent export `.json` remains a report, not the metadata read at runtime.

SuperMimic also uses its own checked sensor scaling: angular velocity `.5` and
joint velocity `.2`; the generic YAML `ang_vel_scale` and `dof_vel_scale` are not
the source for these SuperMimic observation terms.

### Motion window settings

The model requires 20 ms policy periods and future steps `[0,1,2,3,4]`; these
are checked, including `simulation-dt * control-decimation`. Source clock,
500 ms warmup, buffer status handling and generation resets follow the shared
[motion buffer timing](motion_buffer_timing.md).

`default_root_pos: [0,0,0.65]` is the standing fallback reference. It does not
override the simulation's explicit initial base position/quaternion.
`reference_height_offset: 0.05` preserves the existing Pico height correction:
it is added to received base Z only. For an already calibrated training NPZ,
set it to `0.0` in the test configuration. This is a geometric offset, separate
from the observation's scale factors. Other modes keep their current 5 cm
correction.

## Validation

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

The metadata test checks real model loading, name bijections, default/PD
reordering, target scatter and rejection of incompatible models/contracts.

For independent numerical validation, set
`JAKA_SUPER_POLICY_DIAG_JSONL=/absolute/path/policy.jsonl` on a bounded run.
This optional diagnostic records the measured state, sampled window, packed
input, raw output and target. It is disabled in normal deployment. Execute
`tests/check_supermimic_runtime.py` using the training repository's existing
Python environment:

```bash
/path/to/training/.venv/bin/python tests/check_supermimic_runtime.py \
  --training-root /path/to/training \
  --model export/khan_mini/ckpt_0100000.onnx \
  --tape /absolute/path/policy.jsonl \
  --checkpoint /path/to/ckpt_0100000.pt \
  --output /absolute/path/parity.json
```

The checker calls the training observation functions to reconstruct all 620
values, compares ONNX actions and target scattering, and optionally compares
the original PyTorch checkpoint. It does not require a training environment
for ordinary C++ deployment.

Local validation on 2026-11-02 passed all four CTest groups. A 10-second
training walk replay and an 8-second Pico NPZ replay ran with the viewer,
following standing/initialization periods. All 759 and 655 recorded control
ticks were independently checked: maximum observation error was 6.184e-7,
ONNX action and target-scatter errors were zero, and the maximum action
difference against the original PyTorch checkpoint was 3.338e-6. Both short
replays had zero buffer underruns/stale windows and exited with `err=0`.
The public launcher, default 0.60 m simulation initialization, explicit
`--direct-fsm`, legacy ZMQ mode and incompatible timing rejection were also
checked. These are deployment smoke/parity checks, not a full motion-library
benchmark or hardware validation.

The SDK still simulates the parallel ankle mechanism; the training model uses
direct PR actuators. Their physical dynamics are not identical. The SDK's
right hip-roll upper limit is also lower than the training XML's value. Correct
tensor/joint adaptation does not remove these existing model differences.
