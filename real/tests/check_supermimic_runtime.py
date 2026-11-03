#!/usr/bin/env python3
r"""Check a real-deployment SuperMimic tape against training code on CPU.

This is an optional diagnostic, not a default CTest dependency. Run it with the
training repository's existing Python environment. The tape must begin at a
history reset, contain every subsequent policy tick, and use angle_map=0 (the
recorded targets must still be logical joint angles). It never starts a simulator.

Example::

    /path/to/training/.venv/bin/python check_supermimic_runtime.py \
        --training-root /path/to/training --model export/khan_mini/ckpt_0100000.onnx \
        --config /path/to/real/config/KpiConfig.yaml \
        --tape /tmp/super-policy.jsonl --output /tmp/super-parity.json

The two wire name lists below describe the SDK and legacy Isaac motion protocol;
all permutations are independently derived by name, never read from the C++
adapter. Observation formulas come from the actual training repository.
"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

# Do not leave bytecode files in the training checkout during a diagnostic.
sys.dont_write_bytecode = True


DEPLOY_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint",
    "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint",
)

BUFFER_JOINT_NAMES = (
    "Left_hip_pitch_joint", "Right_hip_pitch_joint", "waist_yaw_joint",
    "Left_hip_roll_joint", "Right_hip_roll_joint", "Left_shoulder_pitch_joint",
    "Neck_yaw_joint", "Right_shoulder_pitch_joint", "Left_hip_yaw_joint",
    "Right_hip_yaw_joint", "Left_shoulder_roll_joint", "Neck_pitch_joint",
    "Right_shoulder_roll_joint", "Left_knee_joint", "Right_knee_joint",
    "Left_shoulder_yaw_joint", "Right_shoulder_yaw_joint", "Left_ankle_pitch_joint",
    "Right_ankle_pitch_joint", "Left_elbow_joint", "Right_elbow_joint",
    "Left_ankle_roll_joint", "Right_ankle_roll_joint", "Left_wrist_roll_joint",
    "Right_wrist_roll_joint", "Left_wrist_yaw_joint", "Right_wrist_yaw_joint",
)

# Half-open ranges of the independently reconstructed 620-value observation.
OBSERVATION_BLOCKS = (
    ("reference_root_displacement_b", 0, 15),
    ("reference_world_z", 15, 20),
    ("reference_joint_pos", 20, 155),
    ("motion_anchor_ori_b", 155, 185),
    ("projected_gravity", 185, 200),
    ("base_ang_vel", 200, 215),
    ("joint_pos", 215, 350),
    ("joint_vel", 350, 485),
    ("actions", 485, 620),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def reject_nonfinite_json(value: str):
    raise ValueError(f"Non-finite JSON number: {value}")


def name_permutation(source_names, destination_names):
    """Return source indices that gather values in destination name order."""
    source = [name.strip().casefold() for name in source_names]
    destination = [name.strip().casefold() for name in destination_names]
    if (len(source) != 27 or len(destination) != 27
            or len(set(source)) != 27 or len(set(destination)) != 27):
        raise ValueError("Each joint name order must contain 27 unique names")
    if set(source) != set(destination):
        raise ValueError(f"Joint-name sets differ: {sorted(set(source) ^ set(destination))}")
    source_index = {name: index for index, name in enumerate(source)}
    return [source_index[name] for name in destination]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--training-root", required=True, type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path, help="Real runtime YAML; no hardware is accessed")
    parser.add_argument("--tape", required=True, type=Path,
                        help="Actual C++ policy diagnostic JSONL; angle_map must be 0")
    parser.add_argument("--checkpoint", type=Path,
                        help="Optional trusted training checkpoint for PackedActor/Torch parity")
    parser.add_argument("--output", required=True, type=Path, help="Write the JSON summary here")
    parser.add_argument("--max-steps", type=int, help="Check only this many policy ticks")
    parser.add_argument("--tol", type=float, default=1e-4, help="Absolute error limit (default: 1e-4)")
    args = parser.parse_args()
    if args.max_steps is not None and args.max_steps < 1:
        parser.error("--max-steps must be positive")
    if not math.isfinite(args.tol) or args.tol <= 0:
        parser.error("--tol must be finite and positive")
    return args


def run(args, summary):
    import numpy as np
    import onnxruntime as ort
    import torch
    import yaml

    training_root = args.training_root.resolve()
    sys.path.insert(0, str(training_root / "src"))
    from supermimic.kinematics.rotations import quat_mul, quat_normalize
    from supermimic.observation_layout import isaac_anchor_orientation, isaac_goal_command
    from supermimic.world_model.heads import analytic_observation, flatten_history_term_major

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    functions = (isaac_goal_command, isaac_anchor_orientation,
                 analytic_observation, flatten_history_term_major)
    summary["training_functions"] = {}
    for function in functions:
        source = Path(inspect.getsourcefile(function)).resolve()
        if not source.is_relative_to(training_root):
            raise ValueError(f"Training import escaped --training-root: {source}")
        summary["training_functions"][function.__name__] = {
            "source": str(source), "sha256": sha256(source),
        }
    summary["versions"] = {"torch": torch.__version__, "onnxruntime": ort.__version__,
                           "numpy": np.__version__}
    summary["model_sha256"] = sha256(args.model)
    summary["tape_sha256"] = sha256(args.tape)
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.model), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    metadata = session.get_modelmeta().custom_metadata_map
    schema = json.loads(metadata["onnx_deployment_schema"], parse_constant=reject_nonfinite_json)
    if (schema.get("observation_layout") != "isaac_khan"
            or schema.get("input_layout") != "packed"
            or schema.get("history") != 5 or schema.get("future_frames") != 5
            or schema.get("dimensions") != {"obs": 620, "goal": 155, "policy_obs": 465, "actions": 27}
            or schema.get("quaternion_order") != "wxyz"
            or metadata.get("root_body_name") != "base_link"
            or metadata.get("anchor_body_name") != "base_link"):
        raise ValueError("The model does not declare the supported Khan/base_link 620/27 contract")
    declared_blocks = [(term["name"], term["packed_start"], term["packed_stop"])
                       for term in schema["goal_terms"] + schema["policy_terms"]]
    if declared_blocks != list(OBSERVATION_BLOCKS):
        raise ValueError("ONNX observation block metadata differs from the training layout")
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if (len(inputs) != 1 or inputs[0].name != "obs" or inputs[0].type != "tensor(float)"
            or len(inputs[0].shape) != 2 or inputs[0].shape[1] != 620
            or len(outputs) != 1 or outputs[0].name != "actions"
            or outputs[0].type != "tensor(float)" or len(outputs[0].shape) != 2
            or outputs[0].shape[1] != 27):
        raise ValueError("Expected one float32 obs[batch,620] input and actions[batch,27] output")

    def array(values, shape, label, dtype=np.float32):
        value = np.asarray(values, dtype=dtype)
        if value.shape != shape:
            raise ValueError(f"{label}: expected shape {shape}, got {value.shape}")
        if not np.isfinite(value).all():
            raise ValueError(f"{label}: NaN or infinite value")
        return value

    def metadata_vector(name):
        return array([float(value) for value in metadata[name].split(",")], (27,), name)

    config = yaml.safe_load(args.config.read_text())
    super_config = config["jakaminimimic_superzmq"]
    policy_names = super_config["joint_names"]
    policy_to_deploy = name_permutation(DEPLOY_JOINT_NAMES, policy_names)
    policy_from_buffer = name_permutation(BUFFER_JOINT_NAMES, policy_names)
    deploy_q0 = array(config["default_angles"], (27,), "YAML default_angles")
    q0 = deploy_q0[policy_to_deploy]
    configured_scale = super_config.get("action_scale", config["action_scale"])
    scale = np.full(27, configured_scale, dtype=np.float32) if isinstance(configured_scale, (float, int)) else array(configured_scale, (27,), "YAML action_scale")
    lower = array(config["down_limit"], (27,), "YAML down_limit", np.float64)
    upper = array(config["up_limit"], (27,), "YAML up_limit", np.float64)
    pr_indices = [4, 5, 16, 17]
    lower[pr_indices] = config.get("pitch_roll_down_limit", [-.52, -.35, -.52, -.35])
    upper[pr_indices] = config.get("pitch_roll_up_limit", [.61, .35, .61, .35])
    summary["config"] = str(args.config.resolve())
    summary["parameter_source"] = "Real YAML defaults/scales/policy joint names and physical limits; ONNX metadata read only by this OFFLINE validation for training schema/source identity"
    summary["yaml_defaults_equal_training_metadata"] = bool(np.array_equal(q0, metadata_vector("default_joint_pos")))
    summary["yaml_scale_equal_training_metadata"] = bool(np.array_equal(scale, metadata_vector("action_scale")))
    summary["hardware_pd_preserved"] = {k: config[k] for k in ["kp", "kd", "kp_pitch_roll", "kd_pitch_roll"]}
    summary["physical_target_clipped_components"] = 0
    summary["joint_mapping"] = {
        "policy_joint_names": policy_names, "deploy_joint_names": list(DEPLOY_JOINT_NAMES),
        "buffer_joint_names": list(BUFFER_JOINT_NAMES),
        "policy_to_deploy": policy_to_deploy, "policy_from_buffer": policy_from_buffer,
        "deploy_to_policy": name_permutation(policy_names, DEPLOY_JOINT_NAMES),
        "buffer_to_policy": name_permutation(policy_names, BUFFER_JOINT_NAMES),
    }
    summary["default_joint_pos_policy"] = q0.tolist()
    summary["action_scale_policy"] = scale.tolist()
    summary["requires_angle_map"] = 0
    summary["action_input"] = "Recorded C++ obs (same input for C++, ORT, and optional Torch)"
    summary["history_previous_action"] = "Previous recorded raw action; zero at every reset"

    actor = None
    if args.checkpoint is not None:
        from supermimic.deployment.onnx_export import PackedActor
        from supermimic.eval.benchmark import load_policy_from_checkpoint

        checkpoint_hash = sha256(args.checkpoint)
        summary["checkpoint_sha256"] = checkpoint_hash
        if metadata.get("checkpoint_sha256") != checkpoint_hash:
            raise ValueError("Checkpoint SHA256 does not match the ONNX source checkpoint")
        policy, _ = load_policy_from_checkpoint(args.checkpoint, "cpu")
        actor = PackedActor(policy).eval()

    metrics = summary["comparisons"] = {}

    def compare(name, expected, observed, step, offset=0):
        expected = np.asarray(expected, dtype=np.float64).reshape(-1)
        observed = np.asarray(observed, dtype=np.float64).reshape(-1)
        if expected.shape != observed.shape or not (np.isfinite(expected).all() and np.isfinite(observed).all()):
            raise ValueError(f"{name}: non-finite result or incompatible result shape")
        delta = np.abs(expected - observed)
        index = int(np.argmax(delta))
        error = float(delta[index])
        metric = metrics.setdefault(name, {"max_abs_error": -1.0, "steps_over_tolerance": 0})
        if error > metric["max_abs_error"]:
            metric.update(max_abs_error=error, worst_step=step, worst_index=index + offset,
                          expected=float(expected[index]), observed=float(observed[index]))
        if error > args.tol:
            metric["steps_over_tolerance"] += 1

    history = deque(maxlen=5)
    previous_action = torch.zeros((1, 27), dtype=torch.float32)
    default_q = torch.from_numpy(q0).reshape(1, 27)
    xi = torch.zeros((1, 31), dtype=torch.float32)
    zero_xyz = torch.zeros((1, 3), dtype=torch.float32)
    statuses = Counter()
    generations = set()
    previous_step = None
    activated_generation = None
    legacy_last_ready = False
    legacy_alignment = None
    summary.update(checked_steps=0, reset_count=0, continuation_count=0,
                   stopped_at_max_steps=False, status_counts={}, generations=[])
    with args.tape.open(encoding="utf-8") as stream, torch.inference_mode():
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            if args.max_steps is not None and summary["checked_steps"] >= args.max_steps:
                summary["stopped_at_max_steps"] = True
                break
            summary["last_line"] = line_number
            record = json.loads(line, parse_constant=reject_nonfinite_json)
            step = record["step"]
            if type(step) is not int or step < 0:
                raise ValueError("step must be a nonnegative integer")
            summary["last_step"] = step
            reset = record["reset_history"]
            if type(reset) is not bool:
                raise ValueError("reset_history must be a JSON boolean")
            if not history and not reset:
                raise ValueError("Tape must begin at reset_history=true to reconstruct all history")
            if previous_step is not None and not reset and step != previous_step + 1:
                raise ValueError("Nonconsecutive policy steps without history reset; tape has missing ticks")
            if type(record["query_steady_ns"]) is not int or record["query_steady_ns"] < 0:
                raise ValueError("query_steady_ns must be a nonnegative integer")
            generation = record["generation"]
            if type(generation) is not int or generation < 0:
                raise ValueError("generation must be a nonnegative integer")
            if "angle_map" in record and record["angle_map"] != 0:
                raise ValueError("Target parity requires angle_map=0")
            status = record["status"]
            if status not in ("buffering", "tracking", "underrun", "stale"):
                raise ValueError(f"Unknown motion-window status: {status!r}")
            ready = status in ("tracking", "underrun")
            legacy = record.get("buffer_backend") == "legacy_arrival"
            if legacy:
                if reset != (previous_step is None):
                    raise ValueError("legacy history resets only on entry, not on first received motion")
                if record.get("reference_ready") is not ready or generation != int(ready):
                    raise ValueError("legacy readiness compatibility fields disagree")
                if legacy_last_ready and not ready:
                    raise ValueError("legacy readiness cannot become false after its first packet")
                expected_alignment_reset = previous_step is None or (ready and not legacy_last_ready)
                if record.get("reset_alignment") is not expected_alignment_reset:
                    raise ValueError("legacy alignment must reset on first tick/first ready without clearing history")
                if record.get("query_time_scope") != "diagnostic_write_not_buffer_query":
                    raise ValueError("legacy diagnostic time must not be described as buffer query time")
                legacy_last_ready = ready
            else:
                # Source-clock buffers activate a generation only when ready.
                activate = ready and generation != activated_generation
                if reset != (previous_step is None or activate):
                    raise ValueError("reset_history disagrees with first-tick/ready-generation activation")
                if activate:
                    activated_generation = generation
            frames = record["frames"]
            if not isinstance(frames, list) or len(frames) != 5:
                raise ValueError("Each policy tick must contain five reference frames")
            root_pos = torch.from_numpy(array([f["root_pos"] for f in frames], (5, 3), "root_pos")).unsqueeze(0)
            root_quat = torch.from_numpy(array([f["root_quat"] for f in frames], (5, 4), "root_quat")).unsqueeze(0)
            ref_q = array([f["joint_pos_buffer"] for f in frames], (5, 27), "joint_pos_buffer")
            ref_q = torch.from_numpy(ref_q[:, policy_from_buffer]).unsqueeze(0)
            robot_quat = torch.from_numpy(array(record["robot_quat"], (4,), "robot_quat")).unsqueeze(0)
            align_quat = torch.from_numpy(array(record["align_quat"], (4,), "align_quat")).reshape(1, 1, 4)
            for label, quaternion in (("root_quat", root_quat), ("robot_quat", robot_quat), ("align_quat", align_quat)):
                if bool((quaternion.norm(dim=-1) < 1e-6).any()):
                    raise ValueError(f"{label}: quaternion has near-zero norm")
            root_quat = quat_normalize(root_quat)
            omega = torch.from_numpy(array(record["omega"], (3,), "omega")).unsqueeze(0)
            q = torch.from_numpy(array(record["q_deploy"], (27,), "q_deploy")[policy_to_deploy]).unsqueeze(0)
            qd = torch.from_numpy(array(record["qd_deploy"], (27,), "qd_deploy")[policy_to_deploy]).unsqueeze(0)
            recorded_obs = array(record["obs"], (620,), "obs")
            actions = array(record["actions"], (27,), "actions")
            targets = array(record["targets_deploy"], (27,), "targets_deploy", np.float64)

            if legacy:
                if expected_alignment_reset:
                    def yaw(quat):
                        quat = np.asarray(quat, dtype=float)
                        quat /= np.linalg.norm(quat)
                        w, x, y, z = quat
                        return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
                    delta = yaw(record["robot_quat"]) - yaw(frames[0]["root_quat"])
                    legacy_alignment = np.array([np.cos(delta/2),0.,0.,np.sin(delta/2)])
                recorded_alignment = np.asarray(record["align_quat"], dtype=float)
                # q and -q represent the same rotation.
                alignment_error = min(np.max(abs(recorded_alignment-legacy_alignment)),
                                      np.max(abs(recorded_alignment+legacy_alignment)))
                if alignment_error > 2.e-6:
                    raise ValueError("legacy yaw alignment changed or did not hold between first-ready events")

            # The reference-relative displacement uses the full original reference
            # rotation. Alignment is applied to reference orientation, on the left.
            goal = isaac_goal_command(root_pos, root_quat, ref_q)
            aligned_ref_quat = quat_normalize(quat_mul(quat_normalize(align_quat).expand_as(root_quat), root_quat))
            orientation = isaac_anchor_orientation(quat_normalize(robot_quat), aligned_ref_quat)
            if reset:
                previous_action = torch.zeros_like(previous_action)
            state = torch.cat((zero_xyz, robot_quat, q, zero_xyz, omega, qd), dim=-1)
            sensor = analytic_observation(state, previous_action, xi, default_q, observation_layout="isaac_khan")
            sensor[:, 3:6] *= float(super_config["ang_vel_scale"]) / .5
            sensor[:, 33:60] *= float(super_config["dof_vel_scale"]) / .2
            if reset:
                history.clear()
                history.extend(sensor.clone() for _ in range(5))
                summary["reset_count"] += 1
            else:
                history.append(sensor)
                summary["continuation_count"] += 1
            sensor_history = flatten_history_term_major(torch.stack(tuple(history)), 27, "isaac_khan")
            rebuilt_obs = torch.cat((goal, orientation, sensor_history), dim=-1).numpy().reshape(620)
            compare("observation_total", rebuilt_obs, recorded_obs, step)
            for name, start, stop in OBSERVATION_BLOCKS:
                compare("observation/" + name, rebuilt_obs[start:stop], recorded_obs[start:stop], step, start)

            # Feeding recorded obs isolates inference parity from observation errors.
            ort_actions = session.run(["actions"], {"obs": recorded_obs.reshape(1, 620)})[0]
            ort_actions = array(ort_actions, (1, 27), "ORT output").reshape(27)
            compare("actions_ort_vs_cpp_same_input", ort_actions, actions, step)
            target_policy = q0.astype(np.float64) + scale.astype(np.float64) * actions.astype(np.float64)
            target_deploy = np.empty(27, dtype=np.float64)
            target_deploy[policy_to_deploy] = target_policy
            clipped_target = np.clip(target_deploy, lower, upper)
            summary["physical_target_clipped_components"] += int(np.count_nonzero(target_deploy != clipped_target))
            compare("targets_scatter_and_real_yaml_limits_vs_cpp", clipped_target, targets, step)
            if actor is not None:
                torch_actions = actor(torch.from_numpy(recorded_obs).reshape(1, 620)).numpy()
                torch_actions = array(torch_actions, (1, 27), "Torch output").reshape(27)
                compare("actions_torch_vs_cpp_same_input", torch_actions, actions, step)
                compare("actions_torch_vs_ort_same_input", torch_actions, ort_actions, step)

            previous_action = torch.from_numpy(actions.copy()).reshape(1, 27)
            previous_step = step
            statuses[status] += 1
            generations.add(generation)
            summary["checked_steps"] += 1
            summary["status_counts"] = dict(statuses)
            summary["generations"] = sorted(generations)
    if summary["checked_steps"] == 0:
        raise ValueError("Tape contains no policy ticks")
    summary["failed_comparisons"] = [name for name, metric in metrics.items() if metric["steps_over_tolerance"]]
    summary["passed"] = not summary["failed_comparisons"]


def main():
    args = parse_args()
    summary = {
        "diagnostic_variant": "Real runtime target-limit adaptation of sim/demo/tests/check_supermimic_runtime.py; no controller startup",
        "passed": False, "absolute_tolerance": args.tol, "device": "cpu",
        "training_root": str(args.training_root.resolve()), "model": str(args.model.resolve()),
        "tape": str(args.tape.resolve()), "max_steps": args.max_steps,
        "checkpoint": str(args.checkpoint.resolve()) if args.checkpoint is not None else None,
    }
    try:
        output = args.output.resolve()
        protected = [args.model.resolve(), args.tape.resolve()]
        if args.checkpoint is not None:
            protected.append(args.checkpoint.resolve())
        if output in protected or output == Path(__file__).resolve():
            raise ValueError("--output must not overwrite the model, tape, checkpoint, or this script")
        run(args, summary)
    except Exception as error:
        summary["passed"] = False
        summary["error"] = f"{type(error).__name__}: {error}"
    # Preserve a machine-readable failure report as well as the nonzero exit code.
    if args.output.resolve() not in {args.model.resolve(), args.tape.resolve(), Path(__file__).resolve(),
                                   args.checkpoint.resolve() if args.checkpoint is not None else None}:
        args.output.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: summary[key] for key in ("passed", "checked_steps", "failed_comparisons", "error")
                      if key in summary}, ensure_ascii=False))
    print(f"Summary: {args.output.resolve()}")
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
