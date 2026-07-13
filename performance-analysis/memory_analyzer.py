#!/usr/bin/env python3
"""Memory analysis tool for Dynawo solver profiling data.

Analyses memory usage patterns from the profiler CSV export:
    - Peak memory per phase (from the phase summary section)
    - Memory leak detection via linear regression on the timestep
      memory time-series
    - Jacobian matrix memory footprint estimation
    - Memory usage timeline plot

CSV format: see profile_parser.py (marker-delimited ``# PHASES`` /
``# TIMESTEPS`` sections, with legacy blank-line-separated fallback).

Usage:
    python memory_analyzer.py <profile.csv> [--output-dir <dir>]
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from profile_parser import parse_profile_csv


# ---------------------------------------------------------------------------
# Peak memory per phase
# ---------------------------------------------------------------------------

def analyse_peak_memory(phase_df):
    """Print a table of peak memory per phase, sorted descending."""
    if phase_df is None or phase_df.empty:
        print("\nNo phase data available for peak memory analysis.")
        return

    if "peak_memory_kb" not in phase_df.columns:
        print("\nNo peak_memory_kb column in phase data.")
        return

    df = phase_df.dropna(subset=["peak_memory_kb"])
    if df.empty:
        print("\nNo peak memory data recorded for any phase.")
        return

    df = df.sort_values("peak_memory_kb", ascending=False)

    print("\n" + "=" * 60)
    print("PEAK MEMORY PER PHASE")
    print("=" * 60)
    print(f"{'Phase':<20s} {'Peak Memory (KB)':>18s} {'Peak Memory (MB)':>18s}")
    print("-" * 60)
    for _, row in df.iterrows():
        kb = row["peak_memory_kb"]
        mb = kb / 1024.0
        print(f"{row['phase']:<20s} {kb:>18.0f} {mb:>18.2f}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# Memory leak detection
# ---------------------------------------------------------------------------

def detect_memory_leak(timestep_df, output_dir=None):
    """Detect memory leaks using linear regression on memory time-series.

    A positive slope indicates growing memory usage. If the slope
    exceeds a threshold relative to the initial memory, it is flagged
    as a potential leak.

    Returns
    -------
    slope_kb_per_s : float or None
        The regression slope in KB per simulation-second, or None if
        no data is available.
    """
    if timestep_df is None or timestep_df.empty:
        print("\nNo timestep data available for memory leak detection.")
        return None

    if "memory_kb" not in timestep_df.columns:
        print("\nNo memory_kb column in timestep data.")
        return None

    mem = pd.to_numeric(timestep_df["memory_kb"], errors="coerce").values.astype(float)
    sim_time = pd.to_numeric(timestep_df["sim_time"], errors="coerce").values.astype(float)

    valid = np.isfinite(mem) & np.isfinite(sim_time)
    mem = mem[valid]
    sim_time = sim_time[valid]

    if len(mem) < 10:
        print("\nToo few timestep data points for reliable leak detection "
              f"({len(mem)} points).")
        return None

    negative_count = int(np.sum(mem < 0))
    if negative_count > 0:
        print(f"\nWarning: {negative_count} negative memory samples found in input; "
              "clipping to 0 KB for analysis.")
        mem = np.clip(mem, 0.0, None)

    # Linear regression: memory_kb = slope * sim_time + intercept
    if np.ptp(sim_time) == 0:
        print("\nCould not fit a memory trend: all timestep records share "
              "the same sim_time value.")
        return None
    coeffs = np.polyfit(sim_time, mem, 1)
    slope = coeffs[0]  # KB per simulation-second
    intercept = coeffs[1]
    if not np.isfinite(slope):
        print("\nCould not fit a memory trend: regression is degenerate "
              "(non-finite slope).")
        return None

    sim_span = sim_time[-1] - sim_time[0]
    initial_mem = mem[0] if mem[0] > 0 else 1.0
    regression_growth_kb = slope * sim_span
    regression_growth_pct = regression_growth_kb / initial_mem * 100.0
    observed_growth_kb = mem[-1] - mem[0]
    observed_growth_pct = observed_growth_kb / initial_mem * 100.0

    print("\n" + "=" * 60)
    print("MEMORY LEAK DETECTION")
    print("=" * 60)
    print(f"  Data points:           {len(mem)}")
    print(f"  Simulation time range: [{sim_time[0]:.4f}, {sim_time[-1]:.4f}]")
    print(f"  Initial memory:        {mem[0]:.0f} KB ({mem[0] / 1024:.2f} MB)")
    print(f"  Final memory:          {mem[-1]:.0f} KB ({mem[-1] / 1024:.2f} MB)")
    print(f"  Regression slope:      {slope:.4f} KB/s")
    print(f"  Regression trend:      {regression_growth_pct:+.2f}% over sim range")
    print(f"  Observed net change:   {observed_growth_pct:+.2f}% over sim range")

    projected_10x_kb = max(0.0, mem[0] + regression_growth_kb * 10.0)
    if slope > 0 and regression_growth_pct > 5.0:
        print("\n  ** POTENTIAL MEMORY LEAK DETECTED **")
        print(f"     Memory trend increased by ~{regression_growth_pct:.1f}% over the simulation.")
        print(f"     At this rate, a 10x longer simulation would use "
              f"~{projected_10x_kb:.0f} KB.")
    elif slope > 0 and regression_growth_pct > 1.0:
        print(f"\n  Minor memory growth detected ({regression_growth_pct:.1f}%). "
              "Monitor for longer runs.")
    elif slope < 0:
        print(f"\n  Memory trend is decreasing ({regression_growth_pct:.1f}%). "
              "No leak indicated by regression.")
    else:
        print("\n  No significant memory leak detected.")

    print("=" * 60)

    # Plot with regression line if output_dir is provided
    if output_dir:
        _plot_leak_detection(sim_time, mem, slope, intercept, output_dir)

    return slope


def _plot_leak_detection(sim_time, mem, slope, intercept, output_dir):
    """Plot memory with regression line overlay."""
    fig, ax = plt.subplots(figsize=(12, 5))
    # Clamp measured memory to non-negative values before plotting
    measured_mb = np.maximum(0.0, mem) / 1024.0
    ax.plot(sim_time, measured_mb, linewidth=0.8, color="purple",
            label="Measured")
    fitted_kb = np.maximum(0.0, slope * sim_time + intercept)
    fitted = fitted_kb / 1024.0
    ax.plot(sim_time, fitted, "--", color="red", linewidth=1.2,
            label=f"Regression (slope={slope:.2f} KB/s)")
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Memory (MB)")
    ax.set_title("Memory Usage with Leak Detection Regression")
    ax.ticklabel_format(useOffset=False, style="plain", axis="y")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = os.path.join(output_dir, "memory_leak_detection.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ---------------------------------------------------------------------------
# Jacobian memory footprint estimation
# ---------------------------------------------------------------------------

def estimate_jacobian_memory(problem_size=None, nnz=None):
    """Estimate memory used by the Jacobian matrix.

    Parameters
    ----------
    problem_size : int, optional
        Number of state variables (size of the DAE system).
    nnz : int, optional
        Number of non-zero entries in the Jacobian (for sparse storage).
        If not given, a dense Jacobian is assumed.

    Returns
    -------
    estimate_kb : float
        Estimated memory in KB.
    """
    if problem_size is None:
        return 0.0

    bytes_per_double = 8

    if nnz is not None and nnz > 0:
        # Sparse CSC/CSR: values (nnz doubles) + row indices (nnz ints)
        # + column pointers (n+1 ints)
        bytes_per_int = 4
        total_bytes = (nnz * bytes_per_double +
                       nnz * bytes_per_int +
                       (problem_size + 1) * bytes_per_int)
        storage = "sparse"
    else:
        # Dense: n x n doubles
        total_bytes = problem_size * problem_size * bytes_per_double
        storage = "dense"
        nnz = problem_size * problem_size

    estimate_kb = total_bytes / 1024.0

    print("\n" + "=" * 60)
    print("JACOBIAN MEMORY FOOTPRINT ESTIMATE")
    print("=" * 60)
    print(f"  Problem size (n):  {problem_size}")
    print(f"  Non-zeros (nnz):   {nnz}")
    print(f"  Storage format:    {storage}")
    print(f"  Estimated memory:  {estimate_kb:.2f} KB ({estimate_kb / 1024:.2f} MB)")
    if storage == "dense":
        print(f"  Note: Dense storage assumed. If the Jacobian is sparse,")
        print(f"        provide --nnz for a more accurate estimate.")
    print("=" * 60)

    return estimate_kb


# ---------------------------------------------------------------------------
# Memory timeline plot
# ---------------------------------------------------------------------------

def plot_memory_timeline(timestep_df, output_dir):
    """Plot memory usage timeline with statistics annotations."""
    if timestep_df is None or timestep_df.empty:
        print("\nSkipping memory timeline plot: no timestep data.")
        return

    if "memory_kb" not in timestep_df.columns:
        print("\nSkipping memory timeline plot: no memory_kb column.")
        return

    sim_time = pd.to_numeric(timestep_df["sim_time"], errors="coerce").values
    mem_kb = pd.to_numeric(timestep_df["memory_kb"], errors="coerce").values
    valid = np.isfinite(sim_time) & np.isfinite(mem_kb)
    sim_time = sim_time[valid]
    mem_mb = mem_kb[valid] / 1024.0

    if len(mem_mb) == 0:
        print("\nSkipping memory timeline plot: no valid data after filtering.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(sim_time, mem_mb, linewidth=0.8, color="purple")

    # Annotate min, max, mean
    mean_mb = mem_mb.mean()
    ax.axhline(mean_mb, color="gray", linestyle="--", linewidth=0.8,
               label=f"Mean: {mean_mb:.2f} MB")

    peak_idx = np.argmax(mem_mb)
    ax.annotate(
        f"Peak: {mem_mb[peak_idx]:.2f} MB",
        xy=(sim_time[peak_idx], mem_mb[peak_idx]),
        xytext=(sim_time[peak_idx], mem_mb[peak_idx] * 1.0001),
        arrowprops=dict(arrowstyle="->", color="red"),
        fontsize=9, color="red",
    )

    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Memory (MB)")
    ax.set_title("Memory Usage Timeline")
    ax.ticklabel_format(useOffset=False, style="plain", axis="y")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = os.path.join(output_dir, "memory_timeline.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Memory analysis tool for Dynawo solver profiling data.",
    )
    parser.add_argument("profile_csv", help="Path to the profiler CSV export.")
    parser.add_argument(
        "--output-dir", default=".",
        help="Directory for output charts (default: current directory).",
    )
    parser.add_argument(
        "--problem-size", type=int, default=None,
        help="Number of state variables (for Jacobian memory estimation).",
    )
    parser.add_argument(
        "--nnz", type=int, default=None,
        help="Number of non-zeros in Jacobian (for sparse memory estimation).",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.profile_csv):
        print(f"Error: file not found: {args.profile_csv}", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Analysing memory in {args.profile_csv} ...")
    phase_df, timestep_df = parse_profile_csv(args.profile_csv)

    # 1. Peak memory per phase
    analyse_peak_memory(phase_df)

    # 2. Memory leak detection
    detect_memory_leak(timestep_df, output_dir=args.output_dir)

    # 3. Jacobian memory estimate (if problem size provided)
    if args.problem_size is not None:
        estimate_jacobian_memory(args.problem_size, args.nnz)

    # 4. Memory timeline plot
    plot_memory_timeline(timestep_df, args.output_dir)

    print(f"\nDone. Outputs saved to {os.path.abspath(args.output_dir)}")


if __name__ == "__main__":
    main()
