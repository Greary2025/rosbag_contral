#!/usr/bin/env python3
"""Visualize two TUM-format trajectories in a single 3D interactive Plotly figure.

TUM format per line (whitespace separated):
    timestamp tx ty tz qx qy qz qw
Lines starting with '#' or '%' are ignored. Blank / malformed lines skipped.

Features:
- Align both trajectories so that the first one's initial position becomes the origin.
- Optional decimated orientation arrows.
- Saves interactive HTML output.

Usage:
    ./visualize_two_tum.py --traj1 path/to/traj1.txt --traj2 path/to/traj2.txt \
        --output output.html --arrow-step 50 --arrow-scale 0.5

Dependencies:
    numpy, plotly, numpy-quaternion (optional, only if --arrows using quaternions)
"""
import argparse
import sys
from pathlib import Path
from datetime import datetime
import numpy as np
import plotly.graph_objects as go
import plotly.offline as pyo
from typing import Tuple

try:
    import quaternion  # type: ignore
    _QUAT_AVAILABLE = True
except ImportError:  # allow running without quaternion arrows
    _QUAT_AVAILABLE = False


def load_tum_trajectory(file_path: Path) -> np.ndarray:
    """Load TUM trajectory file into an ndarray of shape (N, 8).

    Returns array columns: [timestamp, tx, ty, tz, qx, qy, qz, qw].
    Skips comment lines (# or %) and malformed lines.
    Raises FileNotFoundError if path missing.
    """
    if not file_path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {file_path}")
    data = []
    with file_path.open('r') as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('%'):
                continue
            parts = line.split()
            if len(parts) != 8:
                # tolerate lines with trailing comments: try first 8 numeric tokens
                numeric_tokens = []
                for token in parts:
                    if len(numeric_tokens) >= 8:
                        break
                    try:
                        float(token)
                        numeric_tokens.append(token)
                    except ValueError:
                        break
                if len(numeric_tokens) != 8:
                    # skip and warn
                    sys.stderr.write(f"Warning: skipping malformed line {line_no} in {file_path}\n")
                    continue
                parts = numeric_tokens
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                sys.stderr.write(f"Warning: non-numeric line {line_no} in {file_path}\n")
                continue
            data.append(vals)
    if not data:
        raise ValueError(f"No valid trajectory data loaded from {file_path}")
    return np.asarray(data, dtype=float)


def align_origin(traj: np.ndarray, other: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Translate trajectories so that first position of traj becomes origin.

    Input arrays shape (N, 8). Translation only; orientation unchanged.
    Returns (traj_aligned, other_aligned) with same shape.
    """
    origin = traj[0, 1:4]  # tx ty tz
    traj_aligned = traj.copy()
    other_aligned = other.copy()
    traj_aligned[:, 1:4] -= origin
    other_aligned[:, 1:4] -= origin
    return traj_aligned, other_aligned


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion (qx,qy,qz,qw) to 3x3 rotation matrix.
    Requires numpy-quaternion. If unavailable, raises RuntimeError.
    """
    if not _QUAT_AVAILABLE:
        raise RuntimeError("numpy-quaternion not installed; cannot render arrows.")
    quat = np.quaternion(q[3], q[0], q[1], q[2])  # w, x, y, z order for library
    return quaternion.as_rotation_matrix(quat)


def create_arrow(position: np.ndarray, rotation_matrix: np.ndarray, scale: float = 0.5):
    direction = rotation_matrix[:, 0]  # x-axis
    end = position + direction * scale
    return position, end


def build_figure(traj1: np.ndarray, traj2: np.ndarray, arrow_step: int, arrow_scale: float, arrows: bool) -> go.Figure:
    fig = go.Figure()

    # Trajectory 1
    fig.add_trace(go.Scatter3d(
        x=traj1[:, 1], y=traj1[:, 2], z=traj1[:, 3],
        mode='lines+markers',
        marker=dict(size=2),
        line=dict(width=2),
        name='Trajectory 1'
    ))

    # Trajectory 2
    fig.add_trace(go.Scatter3d(
        x=traj2[:, 1], y=traj2[:, 2], z=traj2[:, 3],
        mode='lines+markers',
        marker=dict(size=2),
        line=dict(width=2),
        name='Trajectory 2'
    ))

    if arrows and _QUAT_AVAILABLE:
        for i in range(0, traj1.shape[0], arrow_step):
            pos = traj1[i, 1:4]
            q = traj1[i, 4:8]
            R = quaternion_to_rotation_matrix(q)
            start, end = create_arrow(pos, R, arrow_scale)
            fig.add_trace(go.Scatter3d(
                x=[start[0], end[0]], y=[start[1], end[1]], z=[start[2], end[2]],
                mode='lines', line=dict(color='red', width=2), showlegend=False
            ))
        for i in range(0, traj2.shape[0], arrow_step):
            pos = traj2[i, 1:4]
            q = traj2[i, 4:8]
            R = quaternion_to_rotation_matrix(q)
            start, end = create_arrow(pos, R, arrow_scale)
            fig.add_trace(go.Scatter3d(
                x=[start[0], end[0]], y=[start[1], end[1]], z=[start[2], end[2]],
                mode='lines', line=dict(color='green', width=2), showlegend=False
            ))
    elif arrows and not _QUAT_AVAILABLE:
        sys.stderr.write("Warning: numpy-quaternion not installed; skipping arrows.\n")

    fig.update_layout(
        scene=dict(
            aspectmode='data',
            xaxis_title='X', yaxis_title='Y', zaxis_title='Z'
        ),
        title='Two TUM Trajectories (Origin = first traj start)',
        showlegend=True
    )
    return fig


def parse_args():
    p = argparse.ArgumentParser(description='Visualize two TUM trajectories aligned to the first origin.')
    p.add_argument('--traj1', required=True, type=Path, help='Path to first TUM trajectory file.')
    p.add_argument('--traj2', required=True, type=Path, help='Path to second TUM trajectory file.')
    p.add_argument('--output', type=Path, help='Output HTML path (default: alongside traj1 with timestamp).')
    p.add_argument('--arrow-step', type=int, default=0, help='Step interval for orientation arrows (0=disable).')
    p.add_argument('--arrow-scale', type=float, default=0.5, help='Length scale for arrows.')
    return p.parse_args()


def main():
    args = parse_args()

    traj1 = load_tum_trajectory(args.traj1)
    traj2 = load_tum_trajectory(args.traj2)

    traj1_aligned, traj2_aligned = align_origin(traj1, traj2)

    arrows_enabled = args.arrow_step > 0
    fig = build_figure(traj1_aligned, traj2_aligned, args.arrow_step, args.arrow_scale, arrows_enabled)

    # Output path
    if args.output:
        out_path = args.output
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = args.traj1.parent / f"two_traj_vis_{timestamp}.html"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    pyo.plot(fig, filename=str(out_path), auto_open=False)
    print(f"Saved visualization: {out_path}")

    # Optionally show figure in a browser window if environment allows
    try:
        fig.show()
    except Exception:
        print("Note: fig.show() failed (possibly headless environment). HTML saved.")


if __name__ == '__main__':
    main()
