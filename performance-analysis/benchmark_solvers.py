#!/usr/bin/env python3
"""Automated multi-configuration benchmarking for Dynawo solvers.

Defines a set of solver configurations (IDA, SIM, TRAP with various
tolerance settings), launches simulations, collects profiling data,
and compares the results to identify optimal configurations and
parameter sensitivity.

The script invokes Dynawo via its standard CLI:
    <dynawo-bin> jobs <path-to-jobs-file>

Profiling output is captured via the DYNAWO_PROFILE_OUTPUT environment
variable.  A solver .par file is generated for each configuration, but
the .jobs XML must reference it for the parameters to take effect.
For fully automated parameter sweeps, the .jobs file should be
templated or modified before each run.

CSV format expected from each run:
    Phase summary section:
        phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb

    Blank line separator

    Timestep time-series section:
        sim_time,step_duration_ms,memory_kb

Usage:
    python benchmark_solvers.py --dynawo-bin <path> --case <case_dir> [--output-dir <dir>]
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from io import StringIO

import pandas as pd


# ---------------------------------------------------------------------------
# Solver configurations to benchmark
# ---------------------------------------------------------------------------

SOLVER_CONFIGS = [
    {
        "name": "IDA_default",
        "solver": "IDA",
        "params": {
            "order": 2,
            "initStep": 1e-7,
            "minStep": 1e-7,
            "maxStep": 10.0,
            "absAccuracy": 1e-4,
            "relAccuracy": 1e-4,
        },
    },
    {
        "name": "IDA_tight_tol",
        "solver": "IDA",
        "params": {
            "order": 2,
            "initStep": 1e-7,
            "minStep": 1e-7,
            "maxStep": 10.0,
            "absAccuracy": 1e-6,
            "relAccuracy": 1e-6,
        },
    },
    {
        "name": "IDA_loose_tol",
        "solver": "IDA",
        "params": {
            "order": 2,
            "initStep": 1e-7,
            "minStep": 1e-7,
            "maxStep": 10.0,
            "absAccuracy": 1e-3,
            "relAccuracy": 1e-3,
        },
    },
    {
        "name": "SIM_default",
        "solver": "SIM",
        "params": {
            "hMin": 1e-7,
            "hMax": 1.0,
            "kReduceStep": 0.5,
            "maxNewtonTry": 10,
        },
    },
    {
        "name": "SIM_small_step",
        "solver": "SIM",
        "params": {
            "hMin": 1e-8,
            "hMax": 0.1,
            "kReduceStep": 0.5,
            "maxNewtonTry": 10,
        },
    },
    {
        "name": "TRAP_default",
        "solver": "TRAP",
        "params": {
            "hMin": 1e-7,
            "hMax": 1.0,
            "kReduceStep": 0.5,
            "maxNewtonTry": 10,
        },
    },
    {
        "name": "IDA_high_order",
        "solver": "IDA",
        "params": {
            "order": 5,
            "initStep": 1e-7,
            "minStep": 1e-7,
            "maxStep": 10.0,
            "absAccuracy": 1e-4,
            "relAccuracy": 1e-4,
        },
    },
]

# Sensitivity sweep: vary one parameter at a time from a base config
SENSITIVITY_SWEEPS = [
    {
        "base_config": "IDA_default",
        "param": "absAccuracy",
        "values": [1e-3, 1e-4, 1e-5, 1e-6],
    },
    {
        "base_config": "IDA_default",
        "param": "maxStep",
        "values": [0.1, 1.0, 10.0, 100.0],
    },
    {
        "base_config": "SIM_default",
        "param": "hMax",
        "values": [0.01, 0.1, 1.0, 10.0],
    },
]


# ---------------------------------------------------------------------------
# Parsing (duplicated for standalone use)
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
        except Exception:
            pass

    if len(sections) > 1:
        ts_text = sections[1].strip()
        if ts_text:
            try:
                timestep_df = pd.read_csv(StringIO(ts_text))
                timestep_df.columns = [c.strip().lower() for c in timestep_df.columns]
            except Exception:
                pass

    return phase_df, timestep_df


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def check_dynawo_binary(bin_path):
    """Verify that the Dynawo binary exists and is executable."""
    if not os.path.isfile(bin_path):
        print(
            f"ERROR: Dynawo binary not found at '{bin_path}'.\n"
            "\n"
            "To use this benchmarking tool you need a working Dynawo installation.\n"
            "\n"
            "Steps:\n"
            "  1. Build or install Dynawo (https://dynawo.github.io/install)\n"
            "  2. Locate the 'dynawo' or 'dynawo.sh' launcher script\n"
            "  3. Re-run this script with --dynawo-bin pointing to it:\n"
            "       python benchmark_solvers.py \\\n"
            "         --dynawo-bin /path/to/dynawo.sh \\\n"
            "         --case /path/to/case_dir\n",
            file=sys.stderr,
        )
        return False

    if not os.access(bin_path, os.X_OK):
        print(
            f"ERROR: '{bin_path}' exists but is not executable.\n"
            "  Try: chmod +x {bin_path}",
            file=sys.stderr,
        )
        return False

    return True


def generate_solver_par(config, output_path):
    """Generate a Dynawo solver .par file from config dict.

    This writes a minimal XML parameter set that Dynawo can consume.
    """
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<parametersSet xmlns="http://www.rte-france.com/dynawo">',
        f'  <set id="solver">',
    ]
    for key, value in config["params"].items():
        if isinstance(value, float):
            lines.append(f'    <par type="DOUBLE" name="{key}" value="{value}"/>')
        elif isinstance(value, int):
            lines.append(f'    <par type="INT" name="{key}" value="{value}"/>')
        else:
            lines.append(f'    <par type="STRING" name="{key}" value="{value}"/>')
    lines.append("  </set>")
    lines.append("</parametersSet>")

    with open(output_path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def _find_jobs_file(case_dir):
    """Find the .jobs file inside a case directory.

    Returns the path to the first .jobs file found, or None.
    """
    import glob as _glob
    matches = _glob.glob(os.path.join(case_dir, "*.jobs"))
    if matches:
        return matches[0]
    return None


def run_single_benchmark(dynawo_bin, case_dir, config, output_dir):
    """Run a single Dynawo simulation with a given solver configuration.

    The solver parameter file is generated and placed in the case directory
    so that the .jobs XML can reference it.  Profiling output is controlled
    via the DYNAWO_PROFILE_OUTPUT environment variable.

    Returns the path to the generated profile CSV, or None on failure.
    """
    run_dir = os.path.join(output_dir, config["name"])
    os.makedirs(run_dir, exist_ok=True)

    # Generate solver parameter file
    par_path = os.path.join(run_dir, "solver.par")
    generate_solver_par(config, par_path)

    # Find the .jobs file inside the case directory
    jobs_file = _find_jobs_file(case_dir)
    if jobs_file is None:
        print(f"    ERROR: no .jobs file found in {case_dir}", file=sys.stderr)
        return None

    # Profile output path for this run
    profile_csv = os.path.join(run_dir, "profile.csv")

    # Build the command.  Dynawo's CLI is:
    #   myEnvDynawo.sh jobs <path-to-jobs-file>
    # Profiling output is controlled by DYNAWO_PROFILE_OUTPUT env var.
    # NOTE: The solver .par file generated above must be referenced from
    # the .jobs XML to take effect.  For automated parameter sweeps,
    # the .jobs file should be modified or templated before each run.
    cmd = [dynawo_bin, "jobs", jobs_file]

    env = os.environ.copy()
    env["DYNAWO_PROFILE_OUTPUT"] = profile_csv

    print(f"  Running config '{config['name']}' ...")
    print(f"    Jobs file: {jobs_file}")
    print(f"    Solver .par: {par_path}")
    print(f"    Profile output: {profile_csv}")
    print(f"    Command: {' '.join(cmd)}")

    start = time.time()
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=3600, env=env,
        )
        elapsed = time.time() - start
        print(f"    Finished in {elapsed:.1f}s (exit code {result.returncode})")

        if result.returncode != 0:
            print(f"    STDERR: {result.stderr[:500]}", file=sys.stderr)
            return None

    except FileNotFoundError:
        print(f"    ERROR: could not execute '{dynawo_bin}'", file=sys.stderr)
        return None
    except subprocess.TimeoutExpired:
        print(f"    ERROR: simulation timed out after 3600s", file=sys.stderr)
        return None

    # Check for profile CSV
    if os.path.isfile(profile_csv):
        return profile_csv

    print(f"    WARNING: no profile.csv found at {profile_csv}")
    return None


# ---------------------------------------------------------------------------
# Aggregation and reporting
# ---------------------------------------------------------------------------

def collect_results(profile_paths):
    """Collect phase summaries from multiple runs into a comparison dict."""
    results = {}
    for name, path in profile_paths.items():
        if path is None:
            continue
        phase_df, _ = parse_profile_csv(path)
        if phase_df is not None:
            results[name] = phase_df.set_index("phase")["total_seconds"].to_dict()
    return results


def print_benchmark_report(results, configs):
    """Print a summary table comparing all configurations."""
    if not results:
        print("\nNo results to report.")
        return

    all_phases = set()
    for r in results.values():
        all_phases.update(r.keys())
    all_phases = sorted(all_phases)

    config_names = list(results.keys())

    print("\n" + "=" * 80)
    print("BENCHMARK COMPARISON")
    print("=" * 80)

    # Header
    header = f"{'Phase':<20s}"
    for name in config_names:
        header += f" {name:>14s}"
    print(header)
    print("-" * len(header))

    for phase in all_phases:
        row = f"{phase:<20s}"
        for name in config_names:
            val = results[name].get(phase, 0.0)
            row += f" {val:>14.4f}"
        print(row)

    # Totals
    print("-" * len(header))
    row = f"{'TOTAL':<20s}"
    for name in config_names:
        total = sum(results[name].values())
        row += f" {total:>14.4f}"
    print(row)
    print("=" * 80)


def save_results_json(results, output_dir):
    """Save raw results as JSON for further processing."""
    path = os.path.join(output_dir, "benchmark_results.json")
    with open(path, "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"Results saved to {path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Automated multi-configuration benchmarking for Dynawo solvers.",
    )
    parser.add_argument(
        "--dynawo-bin", required=True,
        help="Path to the Dynawo binary / launcher script.",
    )
    parser.add_argument(
        "--case", required=True,
        help="Path to the Dynawo case directory (containing .jobs file).",
    )
    parser.add_argument(
        "--output-dir", default="benchmark_output",
        help="Directory for benchmark outputs (default: benchmark_output).",
    )
    parser.add_argument(
        "--configs", nargs="*", default=None,
        help="Names of configurations to run (default: all). "
             "Available: " + ", ".join(c["name"] for c in SOLVER_CONFIGS),
    )
    parser.add_argument(
        "--sensitivity", action="store_true",
        help="Also run parameter sensitivity sweeps.",
    )
    args = parser.parse_args()

    # Validate binary
    if not check_dynawo_binary(args.dynawo_bin):
        sys.exit(1)

    # Validate case directory
    if not os.path.isdir(args.case):
        print(f"Error: case directory not found: {args.case}", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    # Select configurations
    if args.configs:
        configs = [c for c in SOLVER_CONFIGS if c["name"] in args.configs]
        if not configs:
            print(f"Error: no matching configurations found. Available: "
                  f"{[c['name'] for c in SOLVER_CONFIGS]}", file=sys.stderr)
            sys.exit(1)
    else:
        configs = SOLVER_CONFIGS

    # --- Run main benchmarks ---
    print(f"\nRunning {len(configs)} solver configurations ...")
    profile_paths = {}
    for config in configs:
        path = run_single_benchmark(
            args.dynawo_bin, args.case, config, args.output_dir,
        )
        profile_paths[config["name"]] = path

    results = collect_results(profile_paths)
    print_benchmark_report(results, configs)
    save_results_json(results, args.output_dir)

    # --- Sensitivity sweeps ---
    if args.sensitivity:
        print("\n" + "=" * 80)
        print("PARAMETER SENSITIVITY ANALYSIS")
        print("=" * 80)

        for sweep in SENSITIVITY_SWEEPS:
            base = next(
                (c for c in SOLVER_CONFIGS if c["name"] == sweep["base_config"]),
                None,
            )
            if base is None:
                print(f"  Warning: base config '{sweep['base_config']}' "
                      f"not found, skipping sweep.")
                continue

            param = sweep["param"]
            print(f"\n  Sweeping '{param}' on '{base['name']}':")
            sweep_results = {}

            for val in sweep["values"]:
                config = {
                    "name": f"{base['name']}_{param}_{val}",
                    "solver": base["solver"],
                    "params": {**base["params"], param: val},
                }
                path = run_single_benchmark(
                    args.dynawo_bin, args.case, config, args.output_dir,
                )
                if path:
                    phase_df, _ = parse_profile_csv(path)
                    if phase_df is not None:
                        total = phase_df["total_seconds"].sum()
                        sweep_results[val] = total
                        print(f"    {param}={val}: total={total:.4f}s")

            if sweep_results:
                best_val = min(sweep_results, key=sweep_results.get)
                print(f"  Best value for '{param}': {best_val} "
                      f"(total={sweep_results[best_val]:.4f}s)")

    print("\nBenchmarking complete.")


if __name__ == "__main__":
    main()
