#!/usr/bin/env python3
"""Comprehensive profiling data analysis for Dynawo solver performance.

Parses the CSV export from Dynawo's profiler (phase summary and timestep
time-series sections) and generates visualizations and statistics.

Generated outputs:
    - time_breakdown_pie.png   : Pie chart of phase time distribution
    - hotspot_bar.png          : Bar chart of top phases by total time
    - step_duration_ts.png     : Step duration vs simulation time
    - memory_usage_ts.png      : Memory usage over simulation time
    - Console summary with call-count statistics table

CSV format expected:
    Phase summary section (header row then data rows):
        phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb

    Blank line separator

    Timestep time-series section (header row then data rows):
        sim_time,step_duration_ms,memory_kb

Usage:
    python analyze_profile.py <profile.csv> [--output-dir <dir>]
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_profile_csv(filepath):
    """Parse a Dynawo profiler CSV export.

    Returns
    -------
    phase_df : pd.DataFrame or None
        Phase summary data.
    timestep_df : pd.DataFrame or None
        Timestep time-series data.
    """
    with open(filepath, "r") as fh:
        raw = fh.read()

    # Split on the first blank line to separate the two sections.
    sections = raw.split("\n\n", 1)

    phase_df = None
    timestep_df = None

    # --- Phase summary section ---
    phase_text = sections[0].strip()
    if phase_text:
        from io import StringIO
        try:
            phase_df = pd.read_csv(StringIO(phase_text))
            # Normalise column names (strip whitespace, lowercase)
            phase_df.columns = [c.strip().lower() for c in phase_df.columns]
        except Exception as exc:
            print(f"Warning: could not parse phase summary section: {exc}",
                  file=sys.stderr)

    # --- Timestep time-series section ---
    if len(sections) > 1:
        ts_text = sections[1].strip()
        if ts_text:
            from io import StringIO
            try:
                timestep_df = pd.read_csv(StringIO(ts_text))
                timestep_df.columns = [c.strip().lower() for c in timestep_df.columns]
            except Exception as exc:
                print(f"Warning: could not parse timestep section: {exc}",
                      file=sys.stderr)

    return phase_df, timestep_df


# ---------------------------------------------------------------------------
# Chart generation
# ---------------------------------------------------------------------------

def plot_time_breakdown_pie(phase_df, output_dir):
    """Pie chart of phase time distribution."""
    if phase_df is None or phase_df.empty:
        print("Skipping pie chart: no phase data available.")
        return

    df = phase_df.sort_values("total_seconds", ascending=False)
    # Collapse phases with < 2% share into "Other"
    total = df["total_seconds"].sum()
    if total <= 0:
        print("Skipping pie chart: total time is zero.")
        return

    threshold = 0.02 * total
    main = df[df["total_seconds"] >= threshold]
    other_sum = df[df["total_seconds"] < threshold]["total_seconds"].sum()

    labels = list(main["phase"])
    sizes = list(main["total_seconds"])
    if other_sum > 0:
        labels.append("Other")
        sizes.append(other_sum)

    fig, ax = plt.subplots(figsize=(8, 8))
    wedges, texts, autotexts = ax.pie(
        sizes, labels=labels, autopct="%1.1f%%", startangle=140,
        textprops={"fontsize": 9},
    )
    ax.set_title("Time Breakdown by Phase")
    fig.tight_layout()
    path = os.path.join(output_dir, "time_breakdown_pie.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_hotspot_bar(phase_df, output_dir, top_n=10):
    """Bar chart of top N phases by total time."""
    if phase_df is None or phase_df.empty:
        print("Skipping hotspot bar chart: no phase data available.")
        return

    df = phase_df.nlargest(top_n, "total_seconds")

    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.barh(df["phase"], df["total_seconds"], color="steelblue")
    ax.set_xlabel("Total Time (s)")
    ax.set_title(f"Top {min(top_n, len(df))} Hotspot Phases")
    ax.invert_yaxis()

    for bar, val in zip(bars, df["total_seconds"]):
        ax.text(bar.get_width() + 0.01 * df["total_seconds"].max(),
                bar.get_y() + bar.get_height() / 2,
                f"{val:.3f}s", va="center", fontsize=8)

    fig.tight_layout()
    path = os.path.join(output_dir, "hotspot_bar.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_step_duration_ts(timestep_df, output_dir):
    """Time-series of step duration vs simulation time."""
    if timestep_df is None or timestep_df.empty:
        print("Skipping step duration time-series: no timestep data available.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(timestep_df["sim_time"], timestep_df["step_duration_ms"],
            linewidth=0.8, color="darkorange")
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Step Duration (ms)")
    ax.set_title("Step Duration over Simulation Time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = os.path.join(output_dir, "step_duration_ts.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_memory_usage_ts(timestep_df, output_dir):
    """Memory usage over simulation time."""
    if timestep_df is None or timestep_df.empty:
        print("Skipping memory usage time-series: no timestep data available.")
        return

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(timestep_df["sim_time"], timestep_df["memory_kb"] / 1024.0,
            linewidth=0.8, color="purple")
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Memory (MB)")
    ax.set_title("Memory Usage over Simulation Time")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    path = os.path.join(output_dir, "memory_usage_ts.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"  Saved {path}")


# ---------------------------------------------------------------------------
# Console summary
# ---------------------------------------------------------------------------

def print_summary(phase_df, timestep_df):
    """Print a console summary with call-count statistics."""
    print("\n" + "=" * 70)
    print("PROFILING SUMMARY")
    print("=" * 70)

    if phase_df is not None and not phase_df.empty:
        total_time = phase_df["total_seconds"].sum()
        print(f"\nTotal profiled time: {total_time:.3f} s")
        print(f"Number of phases:   {len(phase_df)}")

        print(f"\n{'Phase':<20s} {'Total(s)':>10s} {'Calls':>10s} "
              f"{'Avg(ms)':>10s} {'Min(ms)':>10s} {'Max(ms)':>10s} "
              f"{'PeakMem(KB)':>12s}")
        print("-" * 92)
        for _, row in phase_df.sort_values("total_seconds",
                                           ascending=False).iterrows():
            peak_mem = row.get("peak_memory_kb", float("nan"))
            peak_str = f"{peak_mem:>12.0f}" if pd.notna(peak_mem) else f"{'N/A':>12s}"
            print(f"{row['phase']:<20s} {row['total_seconds']:>10.4f} "
                  f"{int(row['call_count']):>10d} "
                  f"{row['avg_ms']:>10.3f} {row['min_ms']:>10.3f} "
                  f"{row['max_ms']:>10.3f} {peak_str}")
    else:
        print("\nNo phase summary data available.")

    if timestep_df is not None and not timestep_df.empty:
        print(f"\nTimestep statistics:")
        print(f"  Total timesteps:       {len(timestep_df)}")
        print(f"  Sim time range:        "
              f"[{timestep_df['sim_time'].min():.4f}, "
              f"{timestep_df['sim_time'].max():.4f}]")
        print(f"  Step duration (ms):    "
              f"mean={timestep_df['step_duration_ms'].mean():.3f}, "
              f"median={timestep_df['step_duration_ms'].median():.3f}, "
              f"max={timestep_df['step_duration_ms'].max():.3f}")
        print(f"  Memory (KB):           "
              f"min={timestep_df['memory_kb'].min():.0f}, "
              f"max={timestep_df['memory_kb'].max():.0f}")
    else:
        print("\nNo timestep time-series data available.")

    print("=" * 70)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Analyse Dynawo solver profiling data and generate charts.",
    )
    parser.add_argument("profile_csv", help="Path to the profiler CSV export.")
    parser.add_argument(
        "--output-dir", default=".",
        help="Directory for output charts (default: current directory).",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.profile_csv):
        print(f"Error: file not found: {args.profile_csv}", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Parsing {args.profile_csv} ...")
    phase_df, timestep_df = parse_profile_csv(args.profile_csv)

    print("Generating charts ...")
    plot_time_breakdown_pie(phase_df, args.output_dir)
    plot_hotspot_bar(phase_df, args.output_dir)
    plot_step_duration_ts(timestep_df, args.output_dir)
    plot_memory_usage_ts(timestep_df, args.output_dir)

    print_summary(phase_df, timestep_df)

    print(f"\nDone. Charts saved to {os.path.abspath(args.output_dir)}")


if __name__ == "__main__":
    main()
