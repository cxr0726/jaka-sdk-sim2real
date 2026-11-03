#!/usr/bin/env python3
"""Compare and visualize recorded Robot State vs Reference Action curves.

Reads parquet episode files recorded by `record_jaka_zmq_real.py`, extracts the
corresponding 30 state elements and action elements (27 JAKA joints + 3 root states),
and plots each state vs action pair in a dedicated subplot within a single large figure.

Usage::

    # Default run on the latest recorded episode:
    python scripts/compare_collect_data.py

    # Specify a parquet file:
    python scripts/compare_collect_data.py data/teleop_jaka_mf_real/20261015-135528/level-0/data/chunk-000/episode_000000.parquet

    # Specify a directory (auto-finds episode_*.parquet):
    python scripts/compare_collect_data.py data/teleop_jaka_mf_real/20261015-135528/level-0

    # Save to file without GUI popup (e.g. on remote server / headless):
    python scripts/compare_collect_data.py --no-show -o comparison.png

    # Also plot the extra 10 action dimensions (anchor vel, pos, quat):
    python scripts/compare_collect_data.py --plot-extra
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Default joint names (fallback if info.json is not present)
DEFAULT_STATE_NAMES = [
    # 27 joints
    "Left_hip_pitch_joint",
    "Left_hip_roll_joint",
    "Left_hip_yaw_joint",
    "Left_knee_joint",
    "Left_ankle_pitch_joint",
    "Left_ankle_roll_joint",
    "Right_hip_pitch_joint",
    "Right_hip_roll_joint",
    "Right_hip_yaw_joint",
    "Right_knee_joint",
    "Right_ankle_pitch_joint",
    "Right_ankle_roll_joint",
    "waist_yaw_joint",
    "Left_shoulder_pitch_joint",
    "Left_shoulder_roll_joint",
    "Left_shoulder_yaw_joint",
    "Left_elbow_joint",
    "Left_wrist_roll_joint",
    "Left_wrist_yaw_joint",
    "Right_shoulder_pitch_joint",
    "Right_shoulder_roll_joint",
    "Right_shoulder_yaw_joint",
    "Right_elbow_joint",
    "Right_wrist_roll_joint",
    "Right_wrist_yaw_joint",
    "Neck_yaw_joint",
    "Neck_pitch_joint",
    # 3 root states
    "root_roll",
    "root_pitch",
    "yaw_vel",
]

DEFAULT_EXTRA_ACTION_NAMES = [
    "anchor_lin_vel_x",
    "anchor_lin_vel_y",
    "anchor_lin_vel_z",
    "anchor_pos_w_x",
    "anchor_pos_w_y",
    "anchor_pos_w_z",
    "anchor_quat_w_w",
    "anchor_quat_w_x",
    "anchor_quat_w_y",
    "anchor_quat_w_z",
]


def resolve_paths(target_path: str, info_path: Optional[str] = None) -> Tuple[Path, Optional[Path]]:
    """Resolve parquet file path and meta/info.json path."""
    p = Path(target_path).expanduser().resolve()
    parquet_file: Optional[Path] = None

    if p.is_file() and p.suffix == ".parquet":
        parquet_file = p
    elif p.is_dir():
        # Search for episode_*.parquet
        candidates = sorted(p.rglob("episode_*.parquet"))
        if candidates:
            parquet_file = candidates[0]
            print(f"[compare] Found parquet file: {parquet_file}")
        else:
            candidates = sorted(p.rglob("*.parquet"))
            if candidates:
                parquet_file = candidates[0]
                print(f"[compare] Found parquet file: {parquet_file}")

    if parquet_file is None or not parquet_file.exists():
        raise FileNotFoundError(f"Cannot find parquet file at: {target_path}")

    # Resolve info.json
    meta_json: Optional[Path] = None
    if info_path:
        mp = Path(info_path).expanduser().resolve()
        if mp.is_file():
            meta_json = mp

    if meta_json is None:
        # Search upwards for meta/info.json
        curr = parquet_file.parent
        while curr != curr.parent:
            candidate = curr / "meta" / "info.json"
            if candidate.is_file():
                meta_json = candidate
                break
            curr = curr.parent

    return parquet_file, meta_json


def load_meta_names(info_json_path: Optional[Path]) -> Tuple[List[str], List[str]]:
    """Load state and action feature names from meta/info.json."""
    if info_json_path and info_json_path.is_file():
        try:
            with open(info_json_path, "r", encoding="utf-8") as f:
                info = json.load(f)
            features = info.get("features", {})
            state_names = features.get("state", {}).get("names")
            action_names = features.get("actions", {}).get("names")
            if state_names and action_names:
                return state_names, action_names
        except Exception as e:
            print(f"[compare] Warning: Failed to parse {info_json_path}: {e}")

    # Fallback
    return DEFAULT_STATE_NAMES, DEFAULT_STATE_NAMES + DEFAULT_EXTRA_ACTION_NAMES


def plot_state_vs_action(
    time_arr: np.ndarray,
    state_mat: np.ndarray,
    action_mat: np.ndarray,
    action_names: List[str],
    episode_name: str,
    output_path: Optional[str] = None,
    show: bool = True,
    dpi: int = 150,
) -> None:
    """Plot 33 subplots in a single 7x5 large figure comparing state vs action.

    Indices 0..29: 27 JAKA joints + 3 root states (State vs Action overlay).
    Indices 30..32: anchor_lin_vel_x, anchor_lin_vel_y, anchor_lin_vel_z (Action curve).
    """
    # 30 state/action pairs + 3 anchor linear velocities
    total_subplots = min(action_mat.shape[1], 33)
    nrows, ncols = 7, 5
    assert total_subplots <= nrows * ncols, f"Too many items to fit in {nrows}x{ncols} grid"

    fig, axes = plt.subplots(nrows, ncols, figsize=(28, 21), sharex=True)
    axes_flat = axes.flatten()

    mae_list = []
    rmse_list = []

    print("\n" + "=" * 84)
    print(f"{'Idx':<4} {'Feature Name':<30} {'State Mean':<12} {'Action Mean':<12} {'MAE':<10} {'MaxErr / Range':<15}")
    print("-" * 84)

    for i in range(total_subplots):
        ax = axes_flat[i]
        a_data = action_mat[:, i]
        name = action_names[i] if i < len(action_names) else f"feature_{i}"

        if i < state_mat.shape[1]:
            # Both state and action exist (indices 0..29)
            s_data = state_mat[:, i]
            err = np.abs(s_data - a_data)
            mae = np.mean(err)
            max_err = np.max(err)
            rmse = np.sqrt(np.mean((s_data - a_data) ** 2))
            mae_list.append(mae)
            rmse_list.append(rmse)

            print(f"{i:<4} {name:<30} {np.mean(s_data):<12.4f} {np.mean(a_data):<12.4f} {mae:<10.4f} {max_err:<15.4f}")

            # Plot both state and action
            ax.plot(time_arr, s_data, color="#1f77b4", linewidth=1.5, label="State (Real)")
            ax.plot(time_arr, a_data, color="#d62728", linestyle="--", linewidth=1.5, label="Action (Ref)")
            ax.set_title(f"[{i:02d}] {name}  (MAE: {mae:.3f})", fontsize=10, fontweight="bold", pad=4)
        else:
            # Action only: anchor_lin_vel_x, y, z (indices 30..32)
            a_mean = np.mean(a_data)
            a_min = np.min(a_data)
            a_max = np.max(a_data)
            range_str = f"[{a_min:.2f}, {a_max:.2f}]"
            print(f"{i:<4} {name:<30} {'N/A':<12} {a_mean:<12.4f} {'N/A':<10} {range_str:<15}")

            # Plot action curve
            ax.plot(time_arr, a_data, color="#d62728", linestyle="--", linewidth=1.5, label="Action (Ref)")
            ax.set_title(f"[{i:02d}] {name}  (Action, m/s)", fontsize=10, fontweight="bold", pad=4, color="#b2182b")

        ax.grid(True, linestyle=":", alpha=0.6)
        ax.tick_params(axis="both", which="major", labelsize=8)

        # Ensure bottom-most plots in each column show x-axis labels
        # Cols 0, 1, 2 end at row 6 (indices 30, 31, 32); Cols 3, 4 end at row 5 (indices 28, 29)
        if i in (30, 31, 32, 28, 29):
            ax.tick_params(labelbottom=True)
            ax.set_xlabel("Time (s)", fontsize=9)

    # Hide extra unused subplots in row 6 (indices 33 and 34)
    for j in range(total_subplots, len(axes_flat)):
        fig.delaxes(axes_flat[j])

    print("=" * 84)
    if mae_list:
        print(f"Overall Mean MAE across {len(mae_list)} paired features (0..29): {np.mean(mae_list):.4f} rad")
        print(f"Overall Mean RMSE across {len(rmse_list)} paired features (0..29): {np.mean(rmse_list):.4f} rad")
    print("=" * 84 + "\n")

    # Add single global legend at top
    handles, labels = axes_flat[0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=2,
        fontsize=12,
        frameon=True,
        shadow=True,
    )

    avg_mae = np.mean(mae_list) if mae_list else 0.0
    fig.suptitle(
        f"Episode: {episode_name} | State vs Action Comparison (Total {len(time_arr)} frames, {time_arr[-1]:.2f}s) | Avg MAE (Joints): {avg_mae:.4f}",
        fontsize=15,
        fontweight="bold",
        y=1.01,
    )

    plt.tight_layout(rect=[0, 0, 1, 0.98])

    if output_path:
        out_p = Path(output_path).resolve()
        out_p.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_p, dpi=dpi, bbox_inches="tight")
        print(f"[compare] Main comparison plot saved to: {out_p}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_extra_actions(
    time_arr: np.ndarray,
    action_mat: np.ndarray,
    action_names: List[str],
    episode_name: str,
    output_path: Optional[str] = None,
    show: bool = True,
    dpi: int = 150,
) -> None:
    """Plot the remaining 7 action dimensions: anchor_pos_w[3] and anchor_quat_w[4]."""
    extra_names = action_names[33:]
    extra_mat = action_mat[:, 33:]
    n_extra = extra_mat.shape[1]
    if n_extra == 0:
        return

    nrows, ncols = 2, 4
    fig, axes = plt.subplots(nrows, ncols, figsize=(20, 7), sharex=True)
    axes_flat = axes.flatten()

    for i in range(n_extra):
        ax = axes_flat[i]
        ax.plot(time_arr, extra_mat[:, i], color="#2ca02c", linewidth=1.5)
        name = extra_names[i] if i < len(extra_names) else f"action_{33 + i}"
        ax.set_title(f"[{33 + i:02d}] {name}", fontsize=10, fontweight="bold")
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.tick_params(axis="both", which="major", labelsize=8)
        if i >= 4 or i == 3:
            ax.tick_params(labelbottom=True)
            ax.set_xlabel("Time (s)", fontsize=9)

    for j in range(n_extra, len(axes_flat)):
        fig.delaxes(axes_flat[j])

    fig.suptitle(
        f"Episode: {episode_name} | Extra Actions [33..{33 + n_extra - 1}] (World Anchor Pos & Quat)",
        fontsize=13,
        fontweight="bold",
    )
    plt.tight_layout()

    if output_path:
        out_p = Path(output_path).resolve()
        out_p.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_p, dpi=dpi, bbox_inches="tight")
        print(f"[compare] Extra actions plot saved to: {out_p}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot State vs Reference Action curves for teleop recorded parquet data."
    )
    parser.add_argument(
        "target",
        nargs="?",
        default="data/teleop_jaka_mf_real/20261015-135528/level-0/data/chunk-000/episode_000000.parquet",
        help="Path to episode_*.parquet file or run directory (default: latest recorded episode)",
    )
    parser.add_argument(
        "--info",
        default=None,
        help="Optional path to meta/info.json (auto-detected if omitted)",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="Path to save comparison image (default: <episode_name>_state_vs_action.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display interactive matplotlib window (headless/server mode)",
    )
    parser.add_argument(
        "--plot-extra",
        action="store_true",
        help="Also plot extra 10 action dimensions (anchor vel, pos, quat) in a second figure",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=150,
        help="Figure DPI for saved image (default: 150)",
    )
    args = parser.parse_args()

    # Resolve input paths
    parquet_path, info_json_path = resolve_paths(args.target, args.info)
    print(f"[compare] Reading parquet file: {parquet_path}")
    if info_json_path:
        print(f"[compare] Using meta info: {info_json_path}")

    # Read dataframe
    df = pd.read_parquet(parquet_path)
    if "state" not in df.columns or "actions" not in df.columns:
        raise ValueError(f"Parquet missing 'state' or 'actions' columns! Columns found: {df.columns.tolist()}")

    state_mat = np.stack(df["state"].values).astype(np.float32)
    action_mat = np.stack(df["actions"].values).astype(np.float32)

    # Time / Frame array
    if "timestamp" in df.columns:
        time_arr = df["timestamp"].values.astype(np.float32)
        if time_arr[0] > 1e9:  # if in nanoseconds
            time_arr = (time_arr - time_arr[0]) * 1e-9
        elif time_arr[0] > 1e6:  # if in milliseconds
            time_arr = (time_arr - time_arr[0]) * 1e-3
    else:
        time_arr = np.arange(len(state_mat), dtype=np.float32)

    # Feature names
    state_names, action_names = load_meta_names(info_json_path)

    episode_name = parquet_path.stem
    parent_dir = parquet_path.parent

    # Determine default output image path if not provided
    main_output = args.output
    if main_output is None:
        main_output = str(parent_dir / f"{episode_name}_state_vs_action.png")

    extra_output = None
    if args.plot_extra:
        extra_output = str(parent_dir / f"{episode_name}_extra_actions.png")

    # Plot 33-element comparison figure (30 state vs action + 3 anchor linear vel actions)
    plot_state_vs_action(
        time_arr=time_arr,
        state_mat=state_mat,
        action_mat=action_mat,
        action_names=action_names,
        episode_name=episode_name,
        output_path=main_output,
        show=not args.no_show,
        dpi=args.dpi,
    )

    # Optionally plot remaining 7 action dimensions (anchor pos, anchor quat)
    if args.plot_extra and action_mat.shape[1] > 33:
        plot_extra_actions(
            time_arr=time_arr,
            action_mat=action_mat,
            action_names=action_names,
            episode_name=episode_name,
            output_path=extra_output,
            show=not args.no_show,
            dpi=args.dpi,
        )


if __name__ == "__main__":
    main()
