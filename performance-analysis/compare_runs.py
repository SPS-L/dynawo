#!/usr/bin/env python3
"""Compare two Dynawo profiling runs and generate an HTML report.

Loads two CSV profiles (baseline and optimised), computes per-phase
timing comparisons and speedup ratios, flags regressions, and produces
a self-contained HTML report with an inline Jinja2 template.

CSV format expected:
    Phase summary section (header row then data rows):
        phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb

    Blank line separator

    Timestep time-series section (header row then data rows):
        sim_time,step_duration_ms,memory_kb

Usage:
    python compare_runs.py <baseline.csv> <optimized.csv> [--output <report.html>]
"""

import argparse
import os
import sys
from io import StringIO

import pandas as pd
from jinja2 import Template


# ---------------------------------------------------------------------------
# Parsing (shared logic with analyze_profile.py, duplicated for standalone use)
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

    sections = raw.split("\n\n", 1)

    phase_df = None
    timestep_df = None

    phase_text = sections[0].strip()
    if phase_text:
        try:
            phase_df = pd.read_csv(StringIO(phase_text))
            phase_df.columns = [c.strip().lower() for c in phase_df.columns]
        except Exception as exc:
            print(f"Warning: could not parse phase summary in {filepath}: {exc}",
                  file=sys.stderr)

    if len(sections) > 1:
        ts_text = sections[1].strip()
        if ts_text:
            try:
                timestep_df = pd.read_csv(StringIO(ts_text))
                timestep_df.columns = [c.strip().lower() for c in timestep_df.columns]
            except Exception as exc:
                print(f"Warning: could not parse timestep section in {filepath}: {exc}",
                      file=sys.stderr)

    return phase_df, timestep_df


# ---------------------------------------------------------------------------
# Comparison logic
# ---------------------------------------------------------------------------

def build_comparison(baseline_df, optimized_df):
    """Build a comparison table between two phase summary DataFrames.

    Returns a list of dicts suitable for the HTML template, sorted by
    absolute time difference (largest regression first, then largest
    improvement).
    """
    if baseline_df is None or optimized_df is None:
        return []

    # Index by phase name
    base = baseline_df.set_index("phase")
    opt = optimized_df.set_index("phase")

    all_phases = sorted(set(base.index) | set(opt.index))

    rows = []
    for phase in all_phases:
        base_time = float(base.loc[phase, "total_seconds"]) if phase in base.index else 0.0
        opt_time = float(opt.loc[phase, "total_seconds"]) if phase in opt.index else 0.0

        base_calls = int(base.loc[phase, "call_count"]) if phase in base.index else 0
        opt_calls = int(opt.loc[phase, "call_count"]) if phase in opt.index else 0

        diff = opt_time - base_time
        if opt_time > 0:
            speedup = base_time / opt_time
        elif base_time > 0:
            speedup = float("inf")
        else:
            speedup = 1.0

        regression = opt_time > base_time and base_time > 0

        rows.append({
            "phase": phase,
            "base_time": base_time,
            "opt_time": opt_time,
            "diff": diff,
            "speedup": speedup,
            "base_calls": base_calls,
            "opt_calls": opt_calls,
            "regression": regression,
        })

    # Sort: regressions first (by diff descending), then improvements
    rows.sort(key=lambda r: (-int(r["regression"]), -abs(r["diff"])))
    return rows


def build_timestep_summary(timestep_df, label):
    """Return a dict summarising a timestep DataFrame."""
    if timestep_df is None or timestep_df.empty:
        return {"label": label, "available": False}
    return {
        "label": label,
        "available": True,
        "count": len(timestep_df),
        "sim_range": (
            float(timestep_df["sim_time"].min()),
            float(timestep_df["sim_time"].max()),
        ),
        "step_mean": float(timestep_df["step_duration_ms"].mean()),
        "step_median": float(timestep_df["step_duration_ms"].median()),
        "step_max": float(timestep_df["step_duration_ms"].max()),
        "mem_min": float(timestep_df["memory_kb"].min()),
        "mem_max": float(timestep_df["memory_kb"].max()),
    }


# ---------------------------------------------------------------------------
# HTML report generation (inline Jinja2 template)
# ---------------------------------------------------------------------------

REPORT_TEMPLATE = Template("""\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Dynawo Profile Comparison Report</title>
<style>
  body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
         margin: 2em; background: #fafafa; color: #222; }
  h1 { border-bottom: 2px solid #0066cc; padding-bottom: 0.3em; }
  h2 { color: #0066cc; margin-top: 1.5em; }
  table { border-collapse: collapse; width: 100%; margin: 1em 0; }
  th, td { border: 1px solid #ddd; padding: 6px 10px; text-align: right; }
  th { background: #0066cc; color: white; }
  td:first-child, th:first-child { text-align: left; }
  tr:nth-child(even) { background: #f0f4f8; }
  .regression { background: #ffe0e0 !important; font-weight: bold; }
  .improvement { background: #e0ffe0 !important; }
  .summary-box { background: white; border: 1px solid #ddd; border-radius: 6px;
                 padding: 1em; margin: 1em 0; display: inline-block;
                 min-width: 280px; vertical-align: top; margin-right: 1em; }
  .summary-box h3 { margin-top: 0; }
  .tag-regression { color: #cc0000; }
  .tag-improvement { color: #009900; }
  footer { margin-top: 3em; font-size: 0.85em; color: #888; }
</style>
</head>
<body>
<h1>Dynawo Profile Comparison Report</h1>
<p><strong>Baseline:</strong> {{ baseline_file }}<br>
   <strong>Optimised:</strong> {{ optimized_file }}</p>

<h2>Phase Timing Comparison</h2>
{% if comparison %}
<table>
<tr>
  <th>Phase</th>
  <th>Baseline (s)</th>
  <th>Optimised (s)</th>
  <th>Diff (s)</th>
  <th>Speedup</th>
  <th>Base Calls</th>
  <th>Opt Calls</th>
  <th>Status</th>
</tr>
{% for row in comparison %}
<tr class="{{ 'regression' if row.regression else ('improvement' if row.speedup > 1.05 else '') }}">
  <td>{{ row.phase }}</td>
  <td>{{ "%.4f"|format(row.base_time) }}</td>
  <td>{{ "%.4f"|format(row.opt_time) }}</td>
  <td>{{ "%+.4f"|format(row.diff) }}</td>
  <td>{{ "%.2f"|format(row.speedup) }}x</td>
  <td>{{ row.base_calls }}</td>
  <td>{{ row.opt_calls }}</td>
  <td>
    {% if row.regression %}
      <span class="tag-regression">REGRESSION</span>
    {% elif row.speedup > 1.05 %}
      <span class="tag-improvement">IMPROVED</span>
    {% else %}
      -
    {% endif %}
  </td>
</tr>
{% endfor %}
</table>

{% set total_base = comparison | map(attribute='base_time') | sum %}
{% set total_opt  = comparison | map(attribute='opt_time')  | sum %}
<p><strong>Overall:</strong>
   Baseline total = {{ "%.4f"|format(total_base) }}s,
   Optimised total = {{ "%.4f"|format(total_opt) }}s,
   Overall speedup = {{ "%.2f"|format(total_base / total_opt if total_opt > 0 else 0) }}x
</p>
{% else %}
<p>No phase data available for comparison.</p>
{% endif %}

<h2>Timestep Summary</h2>
<div>
{% for ts in timestep_summaries %}
<div class="summary-box">
  <h3>{{ ts.label }}</h3>
  {% if ts.available %}
  <table>
    <tr><td>Timesteps</td><td>{{ ts.count }}</td></tr>
    <tr><td>Sim time range</td><td>{{ "%.4f"|format(ts.sim_range[0]) }} &ndash; {{ "%.4f"|format(ts.sim_range[1]) }}</td></tr>
    <tr><td>Step mean (ms)</td><td>{{ "%.3f"|format(ts.step_mean) }}</td></tr>
    <tr><td>Step median (ms)</td><td>{{ "%.3f"|format(ts.step_median) }}</td></tr>
    <tr><td>Step max (ms)</td><td>{{ "%.3f"|format(ts.step_max) }}</td></tr>
    <tr><td>Memory min (KB)</td><td>{{ "%.0f"|format(ts.mem_min) }}</td></tr>
    <tr><td>Memory max (KB)</td><td>{{ "%.0f"|format(ts.mem_max) }}</td></tr>
  </table>
  {% else %}
  <p>No timestep data available.</p>
  {% endif %}
</div>
{% endfor %}
</div>

<h2>Regressions</h2>
{% set regressions = comparison | selectattr('regression') | list %}
{% if regressions %}
<ul>
{% for r in regressions %}
  <li><strong>{{ r.phase }}</strong>: {{ "%.4f"|format(r.base_time) }}s &rarr;
      {{ "%.4f"|format(r.opt_time) }}s ({{ "%+.4f"|format(r.diff) }}s,
      {{ "%.2f"|format(r.speedup) }}x)</li>
{% endfor %}
</ul>
{% else %}
<p>No regressions detected. All phases are equal or faster.</p>
{% endif %}

<footer>Generated by Dynawo compare_runs.py</footer>
</body>
</html>
""")


def generate_report(comparison, baseline_file, optimized_file,
                    ts_summaries, output_path):
    """Render the HTML report and write to disk."""
    html = REPORT_TEMPLATE.render(
        comparison=comparison,
        baseline_file=os.path.basename(baseline_file),
        optimized_file=os.path.basename(optimized_file),
        timestep_summaries=ts_summaries,
    )
    with open(output_path, "w") as fh:
        fh.write(html)
    print(f"Report written to {output_path}")


# ---------------------------------------------------------------------------
# Console output
# ---------------------------------------------------------------------------

def print_comparison(comparison):
    """Print a comparison table to the console."""
    if not comparison:
        print("No phase data available for comparison.")
        return

    print(f"\n{'Phase':<20s} {'Base(s)':>10s} {'Opt(s)':>10s} "
          f"{'Diff(s)':>10s} {'Speedup':>8s} {'Status':>12s}")
    print("-" * 78)
    for row in comparison:
        status = "REGRESSION" if row["regression"] else (
            "IMPROVED" if row["speedup"] > 1.05 else "-")
        print(f"{row['phase']:<20s} {row['base_time']:>10.4f} "
              f"{row['opt_time']:>10.4f} {row['diff']:>+10.4f} "
              f"{row['speedup']:>7.2f}x {status:>12s}")

    total_base = sum(r["base_time"] for r in comparison)
    total_opt = sum(r["opt_time"] for r in comparison)
    overall = total_base / total_opt if total_opt > 0 else 0.0
    print(f"\nOverall: {total_base:.4f}s -> {total_opt:.4f}s "
          f"(speedup {overall:.2f}x)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Compare two Dynawo profiling runs and generate an HTML report.",
    )
    parser.add_argument("baseline_csv", help="Path to baseline profiler CSV.")
    parser.add_argument("optimized_csv", help="Path to optimised profiler CSV.")
    parser.add_argument(
        "--output", default="comparison_report.html",
        help="Output HTML report path (default: comparison_report.html).",
    )
    args = parser.parse_args()

    for path in (args.baseline_csv, args.optimized_csv):
        if not os.path.isfile(path):
            print(f"Error: file not found: {path}", file=sys.stderr)
            sys.exit(1)

    print(f"Loading baseline: {args.baseline_csv}")
    base_phase, base_ts = parse_profile_csv(args.baseline_csv)

    print(f"Loading optimised: {args.optimized_csv}")
    opt_phase, opt_ts = parse_profile_csv(args.optimized_csv)

    comparison = build_comparison(base_phase, opt_phase)
    print_comparison(comparison)

    ts_summaries = [
        build_timestep_summary(base_ts, "Baseline"),
        build_timestep_summary(opt_ts, "Optimised"),
    ]

    generate_report(
        comparison, args.baseline_csv, args.optimized_csv,
        ts_summaries, args.output,
    )


if __name__ == "__main__":
    main()
