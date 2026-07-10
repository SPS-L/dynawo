"""Tests for correct percentage denominators in the analysis tools.

Profiler phase timings are *inclusive* and nested (SimulationLoop contains
SolverSolve contains SolverStep contains JacobianEval, ...), so summing all
phases gives ~2-3x wall-clock time. Percentages must be computed against the
SimulationLoop row (wall time of the main loop), not the column sum.
"""

import pandas as pd
import pytest

import analyze_profile
import bottleneck_detector
import compare_runs
import benchmark_solvers


PHASE_COLUMNS = ["phase", "total_seconds", "call_count", "avg_ms",
                 "min_ms", "max_ms", "peak_memory_kb"]


def make_phase_df(rows):
    return pd.DataFrame(rows, columns=PHASE_COLUMNS)


@pytest.fixture
def nested_profile():
    """Synthetic profile with realistic nesting: wall time is 100 s but the
    column sum is ~464 s. JacobianEval is really 45% of wall time."""
    return make_phase_df([
        ("SimulationLoop", 100.0, 1, 100000.0, 100000.0, 100000.0, 500000),
        ("SolverSolve",     96.0, 1000, 96.0, 10.0, 900.0, 500000),
        ("SolverStep",      95.0, 1000, 95.0, 10.0, 900.0, 500000),
        ("NRSolve",         60.0, 1000, 60.0, 5.0, 500.0, 500000),
        ("KINSOLSolve",     58.0, 1200, 48.3, 5.0, 500.0, 500000),
        ("JacobianEval",    45.0, 400, 112.5, 50.0, 300.0, 500000),
        ("ResidualEval",    10.0, 5000, 2.0, 0.5, 10.0, 500000),
        ("CalculateIC",      2.0, 1, 2000.0, 2000.0, 2000.0, 400000),
        ("IO",               8.0, 1, 8000.0, 8000.0, 8000.0, 500000),
    ])


# ---------------------------------------------------------------------------
# bottleneck_detector
# ---------------------------------------------------------------------------

class TestBottleneckDetector:
    def test_wall_time_is_simulation_loop_row(self, nested_profile):
        assert bottleneck_detector.get_wall_time(nested_profile) == 100.0

    def test_wall_time_falls_back_to_sum_without_simulation_loop(self):
        df = make_phase_df([
            ("JacobianEval", 40.0, 400, 100.0, 50.0, 300.0, 0),
            ("ResidualEval", 10.0, 5000, 2.0, 0.5, 10.0, 0),
        ])
        assert bottleneck_detector.get_wall_time(df) == 50.0

    def test_hotspot_percentage_uses_wall_time(self, nested_profile):
        findings = []
        bottleneck_detector.check_dominant_hotspots(nested_profile, findings)
        jac = [f for f in findings if f["phase"] == "JacobianEval"]
        assert len(jac) == 1
        # 45.0 s of a 100.0 s SimulationLoop
        assert "45.0%" in jac[0]["message"]
        assert jac[0]["severity"] == bottleneck_detector.CRITICAL

    def test_hotspot_skips_container_phases(self, nested_profile):
        findings = []
        bottleneck_detector.check_dominant_hotspots(nested_profile, findings)
        flagged = {f["phase"] for f in findings}
        assert "SimulationLoop" not in flagged
        assert "SolverSolve" not in flagged
        assert "SolverStep" not in flagged

    def test_io_overhead_uses_wall_time(self, nested_profile):
        findings = []
        bottleneck_detector.check_io_overhead(nested_profile, findings)
        # IO is 8 s of a 100 s SimulationLoop = 8%
        assert len(findings) == 1
        assert "8.0%" in findings[0]["message"]


# ---------------------------------------------------------------------------
# analyze_profile pie chart data
# ---------------------------------------------------------------------------

class TestPieChartData:
    def test_pie_excludes_container_phases(self, nested_profile):
        labels, sizes = analyze_profile.prepare_pie_data(nested_profile)
        assert "SimulationLoop" not in labels
        assert "SolverSolve" not in labels
        assert "SolverStep" not in labels
        assert "NRSolve" not in labels
        assert "KINSOLSolve" not in labels

    def test_pie_keeps_leaf_phases(self, nested_profile):
        labels, sizes = analyze_profile.prepare_pie_data(nested_profile)
        assert "JacobianEval" in labels
        assert sizes[labels.index("JacobianEval")] == 45.0

    def test_pie_jacobian_share_reflects_wall_fraction(self, nested_profile):
        labels, sizes = analyze_profile.prepare_pie_data(nested_profile)
        # Leaf wedges: Jacobian 45 + Residual 10 + CalculateIC 2 + IO 8 = 65.
        # Jacobian's wedge share (45/65 = 69%) still overstates, so an
        # "Unattributed" remainder wedge must bring the total to wall time.
        assert "Unattributed" in labels
        total = sum(sizes)
        assert total == pytest.approx(100.0)
        assert sizes[labels.index("JacobianEval")] / total == pytest.approx(0.45)


# ---------------------------------------------------------------------------
# compare_runs overall speedup
# ---------------------------------------------------------------------------

class TestCompareRuns:
    def _dfs(self):
        base = make_phase_df([
            ("SimulationLoop", 100.0, 1, 0, 0, 0, 0),
            ("SolverSolve", 96.0, 1000, 0, 0, 0, 0),
            ("JacobianEval", 45.0, 400, 0, 0, 0, 0),
        ])
        opt = make_phase_df([
            ("SimulationLoop", 50.0, 1, 0, 0, 0, 0),
            ("SolverSolve", 48.0, 1000, 0, 0, 0, 0),
            ("JacobianEval", 10.0, 100, 0, 0, 0, 0),
        ])
        return base, opt

    def test_wall_times_from_simulation_loop(self):
        base, opt = self._dfs()
        assert compare_runs.wall_times(base, opt) == (100.0, 50.0)

    def test_html_overall_speedup_is_wall_clock_ratio(self, tmp_path):
        base, opt = self._dfs()
        comparison = compare_runs.build_comparison(base, opt)
        base_wall, opt_wall = compare_runs.wall_times(base, opt)
        out = tmp_path / "report.html"
        compare_runs.generate_report(
            comparison, "b.csv", "o.csv",
            [{"label": "Baseline", "available": False},
             {"label": "Optimised", "available": False}],
            str(out), base_wall, opt_wall,
        )
        html = out.read_text()
        # Wall-clock ratio 100/50 = 2.00x; 2.23x is the nested-sum ratio
        # (241/108), which must not appear.
        assert "2.00" in html
        assert "2.23" not in html


# ---------------------------------------------------------------------------
# benchmark_solvers totals
# ---------------------------------------------------------------------------

class TestBenchmarkTotals:
    def test_total_uses_simulation_loop_when_present(self):
        phases = {"SimulationLoop": 100.0, "SolverSolve": 96.0,
                  "JacobianEval": 45.0}
        assert benchmark_solvers.wall_time_from_phases(phases) == 100.0

    def test_total_falls_back_to_sum(self):
        phases = {"JacobianEval": 45.0, "ResidualEval": 10.0}
        assert benchmark_solvers.wall_time_from_phases(phases) == 55.0
