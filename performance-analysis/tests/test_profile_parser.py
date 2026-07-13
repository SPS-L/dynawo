"""Tests for the shared profile CSV parser and compare_runs status logic."""

import pytest

import compare_runs
from profile_parser import parse_profile_csv

PHASE_SECTION = (
    "phase,total_seconds,call_count,avg_ms,min_ms,max_ms,peak_memory_kb\n"
    "SimulationLoop,100.0,1,100000.0,100000.0,100000.0,500000\n"
    "JacobianEval,45.0,400,112.5,50.0,300.0,500000\n"
)
TS_SECTION = (
    "sim_time,step_duration_ms,memory_kb\n"
    "0.1,5.0,400000\n"
    "0.2,6.0,400100\n"
)


def _write(tmp_path, content):
    path = tmp_path / "profile.csv"
    path.write_text(content)
    return str(path)


class TestParser:
    def test_marker_format_both_sections(self, tmp_path):
        path = _write(tmp_path,
                      "# PHASES\n" + PHASE_SECTION +
                      "\n# TIMESTEPS\n" + TS_SECTION)
        phase_df, ts_df = parse_profile_csv(path)
        assert len(phase_df) == 2
        assert len(ts_df) == 2

    def test_timestep_marker_only_keeps_phase_section(self, tmp_path):
        # Phase rows precede an unmarked "# TIMESTEPS" section: they must
        # not be dropped just because "# PHASES" is absent.
        path = _write(tmp_path,
                      PHASE_SECTION + "\n# TIMESTEPS\n" + TS_SECTION)
        phase_df, ts_df = parse_profile_csv(path)
        assert phase_df is not None
        assert len(phase_df) == 2
        assert len(ts_df) == 2

    def test_legacy_format(self, tmp_path):
        path = _write(tmp_path, PHASE_SECTION + "\n" + TS_SECTION)
        phase_df, ts_df = parse_profile_csv(path)
        assert len(phase_df) == 2
        assert len(ts_df) == 2

    def test_crlf_line_endings(self, tmp_path):
        crlf = (PHASE_SECTION + "\n" + TS_SECTION).replace("\n", "\r\n")
        path = _write(tmp_path, crlf)
        phase_df, ts_df = parse_profile_csv(path)
        assert phase_df is not None and len(phase_df) == 2
        assert ts_df is not None and len(ts_df) == 2


class TestCompareRunsStatus:
    def _compare(self, base_rows, opt_rows):
        import pandas as pd
        cols = ["phase", "total_seconds", "call_count", "avg_ms",
                "min_ms", "max_ms", "peak_memory_kb"]
        base = pd.DataFrame(base_rows, columns=cols)
        opt = pd.DataFrame(opt_rows, columns=cols)
        return {r["phase"]: r
                for r in compare_runs.build_comparison(base, opt)}

    def test_noise_slowdown_not_flagged(self):
        rows = self._compare(
            [("JacobianEval", 45.0, 400, 0, 0, 0, 0)],
            [("JacobianEval", 45.001, 400, 0, 0, 0, 0)],
        )
        assert not rows["JacobianEval"]["regression"]

    def test_real_slowdown_flagged(self):
        rows = self._compare(
            [("JacobianEval", 45.0, 400, 0, 0, 0, 0)],
            [("JacobianEval", 50.0, 400, 0, 0, 0, 0)],
        )
        assert rows["JacobianEval"]["regression"]

    def test_phase_only_in_optimized_run_flagged(self):
        rows = self._compare(
            [("JacobianEval", 45.0, 400, 0, 0, 0, 0)],
            [("JacobianEval", 45.0, 400, 0, 0, 0, 0),
             ("KLUSymbolic", 5.0, 10, 0, 0, 0, 0)],
        )
        assert rows["KLUSymbolic"]["regression"]
        assert rows["KLUSymbolic"]["new_phase"]
        assert rows["KLUSymbolic"]["speedup"] is None

    def test_improvement_status(self):
        rows = self._compare(
            [("JacobianEval", 45.0, 400, 0, 0, 0, 0)],
            [("JacobianEval", 10.0, 100, 0, 0, 0, 0)],
        )
        row = rows["JacobianEval"]
        assert not row["regression"]
        assert row["speedup"] == pytest.approx(4.5)
