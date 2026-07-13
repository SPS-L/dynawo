"""Shared parsing helpers for Dynawo profiler CSV exports.

Used by analyze_profile.py, bottleneck_detector.py, compare_runs.py,
memory_analyzer.py and benchmark_solvers.py so that format handling lives
in one place.

Supported formats:
    New format (explicit section markers):
        # PHASES
        phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb
        ...

        # TIMESTEPS
        sim_time,step_duration_ms,memory_kb
        ...

    Legacy format: the same two tables separated by a single blank line,
    without section markers.
"""

import sys
from io import StringIO

import pandas as pd


def _strip_comment_lines(text):
    """Remove lines starting with '#' and return cleaned text."""
    lines = [ln for ln in text.splitlines() if not ln.strip().startswith("#")]
    return "\n".join(lines)


def _df_from_text(text, section_name):
    """Parse a CSV block into a DataFrame, normalising column names."""
    cleaned = _strip_comment_lines(text).strip()
    if not cleaned:
        return None
    try:
        df = pd.read_csv(StringIO(cleaned))
        df.columns = [c.strip().lower() for c in df.columns]
        return df
    except Exception as exc:
        print(f"Warning: could not parse {section_name} section: {exc}",
              file=sys.stderr)
        return None


def _parse_with_markers(raw):
    """Parse new-format CSV that contains '# PHASES' / '# TIMESTEPS' markers."""
    phase_pos = raw.find("# PHASES")
    ts_pos = raw.find("# TIMESTEPS")

    phase_text = ""
    ts_text = ""

    if phase_pos != -1:
        phase_text = raw[phase_pos:ts_pos] if ts_pos != -1 else raw[phase_pos:]
    elif ts_pos != -1:
        # Only the timestep marker is present: everything before it is the
        # (unmarked) phase section.
        phase_text = raw[:ts_pos]

    if ts_pos != -1:
        ts_text = raw[ts_pos:]

    phase_df = _df_from_text(phase_text, "phase summary") if phase_text else None
    timestep_df = _df_from_text(ts_text, "timestep time-series") if ts_text else None
    return phase_df, timestep_df


def _parse_legacy(raw):
    """Parse legacy-format CSV with a blank-line separator between sections."""
    sections = raw.split("\n\n", 1)

    phase_df = _df_from_text(sections[0], "phase summary") if sections else None
    timestep_df = None
    if len(sections) > 1:
        timestep_df = _df_from_text(sections[1], "timestep time-series")

    return phase_df, timestep_df


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

    # Normalise line endings so section splitting works on CRLF files.
    raw = raw.replace("\r\n", "\n").replace("\r", "\n")

    if "# PHASES" in raw or "# TIMESTEPS" in raw:
        return _parse_with_markers(raw)
    return _parse_legacy(raw)


def get_wall_time(phase_df):
    """Wall-clock reference: the SimulationLoop row, else the column sum.

    Phase timings are inclusive and nested, so the SimulationLoop row is the
    only valid percentage/speedup denominator; the column sum is a fallback
    for truncated profiles without that row.
    """
    if phase_df is None or phase_df.empty:
        return 0.0
    matches = phase_df[phase_df["phase"].str.lower() == "simulationloop"]
    if not matches.empty:
        return float(matches.iloc[0]["total_seconds"])
    return float(phase_df["total_seconds"].sum())
