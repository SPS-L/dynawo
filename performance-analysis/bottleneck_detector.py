#!/usr/bin/env python3
"""Automatic bottleneck analysis for Dynawo solver profiling data.

Loads a profiling CSV export, applies a set of heuristic rules to
identify performance bottlenecks, and outputs prioritised
recommendations with severity levels.

Checks performed:
    1. Phases consuming >10% of total time (dominant hotspots)
    2. Excessive Jacobian rebuilds relative to solver steps
    3. Linear solver efficiency (factorisation-to-solve ratio)
    4. Convergence issues (too many KINSOL solves per solver step)
    5. Timestep variability and outlier detection

Severity levels:
    CRITICAL  - Likely the primary performance limiter
    WARNING   - Notable issue worth investigating
    INFO      - Observation that may help with tuning

CSV format expected:
    Phase summary section:
        phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb

    Blank line separator

    Timestep time-series section:
        sim_time,step_duration_ms,memory_kb

Usage:
    python bottleneck_detector.py <profile.csv>
"""

import argparse
import os
import sys
from io import StringIO

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Severity levels
# ---------------------------------------------------------------------------

CRITICAL = "CRITICAL"
WARNING = "WARNING"
INFO = "INFO"

SEVERITY_ORDER = {CRITICAL: 0, WARNING: 1, INFO: 2}


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

def parse_profile_csv(filepath):
    """Parse a Dynawo profiler CSV export."""
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
            print(f"Warning: could not parse phase summary: {exc}",
                  file=sys.stderr)

    if len(sections) > 1:
        ts_text = sections[1].strip()
        if ts_text:
            try:
                timestep_df = pd.read_csv(StringIO(ts_text))
                timestep_df.columns = [c.strip().lower() for c in timestep_df.columns]
            except Exception as exc:
                print(f"Warning: could not parse timestep section: {exc}",
                      file=sys.stderr)

    return phase_df, timestep_df


# ---------------------------------------------------------------------------
# Helper: safe lookup of a phase row
# ---------------------------------------------------------------------------

def get_phase(phase_df, name):
    """Return the row for a given phase name, or None."""
    if phase_df is None:
        return None
    matches = phase_df[phase_df["phase"].str.lower() == name.lower()]
    if matches.empty:
        return None
    return matches.iloc[0]


def get_phase_value(phase_df, name, column, default=0):
    """Return a single value from a phase row."""
    row = get_phase(phase_df, name)
    if row is None:
        return default
    return row.get(column, default)


# ---------------------------------------------------------------------------
# Bottleneck checks
# ---------------------------------------------------------------------------

def check_dominant_hotspots(phase_df, findings):
    """Flag phases that consume >10% of total profiled time."""
    if phase_df is None or phase_df.empty:
        return

    total = phase_df["total_seconds"].sum()
    if total <= 0:
        return

    for _, row in phase_df.iterrows():
        fraction = row["total_seconds"] / total
        if fraction > 0.10:
            pct = fraction * 100
            severity = CRITICAL if fraction > 0.40 else WARNING
            findings.append({
                "severity": severity,
                "category": "Hotspot",
                "phase": row["phase"],
                "message": (
                    f"Phase '{row['phase']}' consumes {pct:.1f}% of total "
                    f"time ({row['total_seconds']:.4f}s / {total:.4f}s)."
                ),
                "recommendation": _hotspot_recommendation(row["phase"]),
            })


def _hotspot_recommendation(phase):
    """Return a phase-specific tuning recommendation."""
    tips = {
        "jacobianeval": (
            "Consider increasing the Jacobian reuse count or switching to a "
            "Jacobian-free method. Check if the model has unnecessary "
            "discontinuities forcing frequent rebuilds."
        ),
        "residualeval": (
            "Profile the model equations for expensive computations. "
            "Look for table lookups or external function calls that "
            "could be simplified."
        ),
        "nrsolve": (
            "This is the full Newton-Raphson algebraic solve for "
            "fixed-timestep solvers (SIM/TRAP). If it dominates, "
            "check JacobianEval and KINSOLSolve sub-phases. Consider "
            "relaxing NR tolerances or improving initial guesses."
        ),
        "solversolve": (
            "This is the overall solver time. Drill down into sub-phases "
            "(JacobianEval, NRSolve, ResidualEval) to find "
            "the root cause."
        ),
        "kinsolsolve": (
            "Many KINSOL solves may indicate convergence difficulty. "
            "Consider adjusting tolerances, initial conditions, or the "
            "algebraic variable handling."
        ),
        "matrixcopy": (
            "Excessive matrix copying suggests the Jacobian reuse strategy "
            "is suboptimal. Consider adjusting jcopyMaxStep or similar "
            "parameters."
        ),
        "calculateic": (
            "Slow initial condition calculation may indicate a poor "
            "starting point. Consider providing better initial guesses "
            "or relaxing IC tolerances."
        ),
        "modeeval": (
            "Mode evaluation detects topology changes and protection "
            "relay actions. If expensive, check for frequent mode "
            "changes or complex model transitions."
        ),
        "io": (
            "Output writing (curves, timeline, IIDM state) is taking "
            "significant time. Consider reducing output frequency or "
            "the number of exported variables."
        ),
    }
    key = phase.lower().replace(" ", "")
    return tips.get(key, "Investigate this phase for optimisation opportunities.")


def check_jacobian_rebuilds(phase_df, findings):
    """Detect excessive Jacobian rebuilds relative to solver steps."""
    jac_calls = get_phase_value(phase_df, "JacobianEval", "call_count", 0)
    solver_steps = get_phase_value(phase_df, "SolverStep", "call_count", 0)

    if solver_steps <= 0 or jac_calls <= 0:
        return

    ratio = jac_calls / solver_steps

    if ratio > 2.0:
        severity = CRITICAL if ratio > 5.0 else WARNING
        findings.append({
            "severity": severity,
            "category": "Jacobian Rebuilds",
            "phase": "JacobianEval",
            "message": (
                f"Jacobian evaluated {jac_calls} times for {solver_steps} "
                f"solver steps (ratio {ratio:.2f}). "
                f"Ideally this should be close to 1.0."
            ),
            "recommendation": (
                "Increase the maximum Jacobian reuse count in the solver "
                "parameters. Check for frequent mode changes or "
                "discontinuities that force rebuilds."
            ),
        })
    elif ratio < 0.5:
        findings.append({
            "severity": INFO,
            "category": "Jacobian Rebuilds",
            "phase": "JacobianEval",
            "message": (
                f"Jacobian evaluated only {jac_calls} times for "
                f"{solver_steps} steps (ratio {ratio:.2f}). "
                f"This is efficient."
            ),
            "recommendation": "No action needed. Jacobian reuse is working well.",
        })


def check_linear_solver_efficiency(phase_df, findings):
    """Check the ratio of NR solves to Jacobian factorisations.

    Uses the NRSolve phase (SIM/TRAP fixed-timestep solvers).
    For IDA profiles this check is skipped since the linear solve
    happens internally within SUNDIALS and has no separate phase.
    """
    linear_calls = get_phase_value(phase_df, "NRSolve", "call_count", 0)
    jac_calls = get_phase_value(phase_df, "JacobianEval", "call_count", 0)

    if jac_calls <= 0 or linear_calls <= 0:
        return

    ratio = linear_calls / jac_calls

    if ratio < 2.0:
        findings.append({
            "severity": WARNING,
            "category": "Linear Solver Efficiency",
            "phase": "NRSolve",
            "message": (
                f"Only {ratio:.1f} NR solves per Jacobian evaluation "
                f"({linear_calls} solves, {jac_calls} factorisations). "
                f"The factorisation cost is not being amortised well."
            ),
            "recommendation": (
                "Increase the Jacobian reuse count so that each "
                "factorisation is used for more NR solves."
            ),
        })
    elif ratio > 50:
        findings.append({
            "severity": INFO,
            "category": "Linear Solver Efficiency",
            "phase": "NRSolve",
            "message": (
                f"High reuse ratio: {ratio:.1f} NR solves per "
                f"Jacobian evaluation. This is efficient but verify "
                f"that convergence is not degraded."
            ),
            "recommendation": (
                "Monitor convergence; very old Jacobians may cause "
                "extra Newton iterations."
            ),
        })


def check_kinsol_convergence(phase_df, findings):
    """Detect convergence issues from excessive KINSOL solves per step."""
    kinsol_calls = get_phase_value(phase_df, "KINSOLSolve", "call_count", 0)
    solver_steps = get_phase_value(phase_df, "SolverStep", "call_count", 0)

    if solver_steps <= 0 or kinsol_calls <= 0:
        return

    ratio = kinsol_calls / solver_steps

    if ratio > 3.0:
        severity = CRITICAL if ratio > 10.0 else WARNING
        findings.append({
            "severity": severity,
            "category": "Convergence",
            "phase": "KINSOLSolve",
            "message": (
                f"KINSOL called {kinsol_calls} times for {solver_steps} "
                f"solver steps (ratio {ratio:.2f}). This suggests "
                f"convergence difficulty."
            ),
            "recommendation": (
                "Check algebraic variable initialisation. Consider "
                "adjusting KINSOL tolerances, scaling, or the strategy "
                "for handling algebraic loops."
            ),
        })


def check_reinit_frequency(phase_df, findings):
    """Flag excessive re-initialisations."""
    reinit_calls = get_phase_value(phase_df, "Reinit", "call_count", 0)
    solver_steps = get_phase_value(phase_df, "SolverStep", "call_count", 0)

    if reinit_calls <= 0:
        return

    if solver_steps > 0:
        ratio = reinit_calls / solver_steps
    else:
        ratio = float("inf")

    if reinit_calls > 10 or (solver_steps > 0 and ratio > 0.1):
        findings.append({
            "severity": WARNING,
            "category": "Re-initialisation",
            "phase": "Reinit",
            "message": (
                f"Simulation re-initialised {reinit_calls} times "
                f"({ratio:.3f} per solver step). Frequent reinits "
                f"can be expensive."
            ),
            "recommendation": (
                "Investigate the cause of re-initialisations. Common "
                "causes include mode changes and discontinuities. "
                "Consider smoothing model transitions."
            ),
        })


def check_timestep_variability(timestep_df, findings):
    """Detect high variability or outliers in timestep durations."""
    if timestep_df is None or timestep_df.empty:
        return

    durations = timestep_df["step_duration_ms"]
    mean_dur = durations.mean()
    std_dur = durations.std()
    max_dur = durations.max()

    if mean_dur <= 0:
        return

    cv = std_dur / mean_dur  # coefficient of variation

    if cv > 2.0:
        findings.append({
            "severity": WARNING,
            "category": "Timestep Variability",
            "phase": "SolverStep",
            "message": (
                f"High variability in step duration: CV={cv:.2f} "
                f"(mean={mean_dur:.3f}ms, std={std_dur:.3f}ms, "
                f"max={max_dur:.3f}ms)."
            ),
            "recommendation": (
                "Some timesteps are much more expensive than others. "
                "Check for events or mode changes at the slow timesteps. "
                "Consider adaptive step-size tuning."
            ),
        })

    # Outlier detection: steps > 10x mean
    outliers = durations[durations > 10 * mean_dur]
    if len(outliers) > 0:
        findings.append({
            "severity": INFO,
            "category": "Timestep Outliers",
            "phase": "SolverStep",
            "message": (
                f"{len(outliers)} timesteps are >10x the mean duration. "
                f"These outliers account for "
                f"{outliers.sum():.1f}ms out of {durations.sum():.1f}ms total."
            ),
            "recommendation": (
                "Identify the simulation times where outliers occur "
                "and correlate with model events."
            ),
        })


def check_io_overhead(phase_df, findings):
    """Flag if I/O is a significant fraction of total time."""
    io_time = get_phase_value(phase_df, "IO", "total_seconds", 0)
    if phase_df is None:
        return
    total = phase_df["total_seconds"].sum()
    if total <= 0 or io_time <= 0:
        return

    fraction = io_time / total
    if fraction > 0.05:
        findings.append({
            "severity": WARNING if fraction > 0.15 else INFO,
            "category": "I/O Overhead",
            "phase": "IO",
            "message": (
                f"I/O accounts for {fraction * 100:.1f}% of total time "
                f"({io_time:.4f}s)."
            ),
            "recommendation": (
                "Reduce output frequency or switch to binary output "
                "format. Consider writing outputs asynchronously."
            ),
        })


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def print_findings(findings):
    """Print sorted findings to the console."""
    if not findings:
        print("\nNo bottlenecks detected. The profile looks healthy.")
        return

    # Sort by severity
    findings.sort(key=lambda f: SEVERITY_ORDER.get(f["severity"], 99))

    print("\n" + "=" * 78)
    print("BOTTLENECK ANALYSIS REPORT")
    print("=" * 78)

    prev_severity = None
    for i, f in enumerate(findings, 1):
        if f["severity"] != prev_severity:
            print(f"\n--- {f['severity']} ---")
            prev_severity = f["severity"]

        print(f"\n  [{i}] {f['category']} ({f['phase']})")
        print(f"      {f['message']}")
        print(f"      Recommendation: {f['recommendation']}")

    print("\n" + "=" * 78)
    counts = {}
    for f in findings:
        counts[f["severity"]] = counts.get(f["severity"], 0) + 1
    parts = [f"{v} {k}" for k, v in sorted(counts.items(),
                                            key=lambda x: SEVERITY_ORDER.get(x[0], 99))]
    print(f"Total findings: {len(findings)} ({', '.join(parts)})")
    print("=" * 78)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Automatic bottleneck analysis for Dynawo profiling data.",
    )
    parser.add_argument("profile_csv", help="Path to the profiler CSV export.")
    args = parser.parse_args()

    if not os.path.isfile(args.profile_csv):
        print(f"Error: file not found: {args.profile_csv}", file=sys.stderr)
        sys.exit(1)

    print(f"Analysing {args.profile_csv} ...")
    phase_df, timestep_df = parse_profile_csv(args.profile_csv)

    if phase_df is None or phase_df.empty:
        print("Error: no phase summary data found in the CSV.", file=sys.stderr)
        sys.exit(1)

    findings = []

    # Run all checks
    check_dominant_hotspots(phase_df, findings)
    check_jacobian_rebuilds(phase_df, findings)
    check_linear_solver_efficiency(phase_df, findings)
    check_kinsol_convergence(phase_df, findings)
    check_reinit_frequency(phase_df, findings)
    check_timestep_variability(timestep_df, findings)
    check_io_overhead(phase_df, findings)

    print_findings(findings)


if __name__ == "__main__":
    main()
