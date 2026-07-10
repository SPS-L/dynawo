"""Tests that benchmark_solvers.py applies each solver configuration by
executing a rewired temporary copy of the case's jobs file."""

import os
import stat
import xml.etree.ElementTree as ET

import pytest

import benchmark_solvers

DYN_NS = "http://www.rte-france.com/dynawo"

MINIMAL_JOBS = """<?xml version='1.0' encoding='UTF-8'?>
<dyn:jobs xmlns:dyn="http://www.rte-france.com/dynawo">
    <dyn:job name="Minimal case">
        <dyn:solver lib="dynawo_SolverSIM" parFile="Solver.par" parId="Solver"/>
        <dyn:modeler compileDir="outputs/compilation">
            <dyn:network iidmFile="Case.iidm" parFile="Network.par" parId="Network"/>
            <dyn:precompiledModels useStandardModels="true"/>
            <dyn:modelicaModels useStandardModels="false"/>
        </dyn:modeler>
        <dyn:simulation startTime="0" stopTime="10"/>
        <dyn:outputs directory="outputs"/>
    </dyn:job>
</dyn:jobs>
"""

IDA_CONFIG = {
    "name": "IDA_test",
    "solver": "IDA",
    "params": {"order": 2, "absAccuracy": 1e-4},
}


@pytest.fixture
def case_dir(tmp_path):
    d = tmp_path / "case"
    d.mkdir()
    (d / "Case.jobs").write_text(MINIMAL_JOBS)
    return d


def _solver_attrs(jobs_path):
    root = ET.parse(jobs_path).getroot()
    solver = root.find(f".//{{{DYN_NS}}}solver")
    assert solver is not None, "no dyn:solver element found"
    return solver.attrib


class TestPrepareJobsFile:
    def test_rewires_solver_element(self, case_dir, tmp_path):
        par_path = tmp_path / "run" / "solver.par"
        par_path.parent.mkdir()
        benchmark_solvers.generate_solver_par(IDA_CONFIG, str(par_path))

        prepared = benchmark_solvers.prepare_jobs_file(
            str(case_dir / "Case.jobs"), IDA_CONFIG, str(par_path))

        attrs = _solver_attrs(prepared)
        assert attrs["lib"] == "dynawo_SolverIDA"
        assert attrs["parFile"] == os.path.abspath(str(par_path))
        # generate_solver_par writes <set id="solver">
        assert attrs["parId"] == "solver"

    def test_prepared_file_lives_next_to_original(self, case_dir, tmp_path):
        """Relative paths in the jobs file (iidm, dyd, par) resolve against
        the jobs file location, so the rewired copy must be in the same
        directory."""
        par_path = tmp_path / "solver.par"
        benchmark_solvers.generate_solver_par(IDA_CONFIG, str(par_path))

        prepared = benchmark_solvers.prepare_jobs_file(
            str(case_dir / "Case.jobs"), IDA_CONFIG, str(par_path))

        assert os.path.dirname(os.path.abspath(prepared)) == str(case_dir)
        assert prepared != str(case_dir / "Case.jobs")

    def test_original_jobs_untouched(self, case_dir, tmp_path):
        par_path = tmp_path / "solver.par"
        benchmark_solvers.generate_solver_par(IDA_CONFIG, str(par_path))
        original = (case_dir / "Case.jobs").read_text()

        benchmark_solvers.prepare_jobs_file(
            str(case_dir / "Case.jobs"), IDA_CONFIG, str(par_path))

        assert (case_dir / "Case.jobs").read_text() == original


class TestRunSingleBenchmark:
    def _fake_dynawo(self, tmp_path, capture_path):
        """A fake Dynawo launcher: records the jobs file it was given and
        writes a minimal profile CSV where DYNAWO_PROFILE_OUTPUT points."""
        script = tmp_path / "fake_dynawo.sh"
        script.write_text(
            "#!/bin/sh\n"
            f'cp "$2" "{capture_path}"\n'
            'printf "phase,total_seconds,call_count,avg_ms,min_ms,max_ms,'
            'peak_memory_kb\\nSimulationLoop,1.0,1,1000.0,1000.0,1000.0,0\\n"'
            ' > "$DYNAWO_PROFILE_OUTPUT"\n'
        )
        script.chmod(script.stat().st_mode | stat.S_IEXEC)
        return str(script)

    def test_dynawo_receives_rewired_jobs_file(self, case_dir, tmp_path):
        capture = tmp_path / "captured.jobs"
        fake_bin = self._fake_dynawo(tmp_path, capture)
        out_dir = tmp_path / "bench_out"

        profile = benchmark_solvers.run_single_benchmark(
            fake_bin, str(case_dir), IDA_CONFIG, str(out_dir))

        assert profile is not None and os.path.isfile(profile)
        attrs = _solver_attrs(capture)
        assert attrs["lib"] == "dynawo_SolverIDA"
        assert attrs["parFile"].endswith("solver.par")

    def test_temporary_jobs_file_is_cleaned_up(self, case_dir, tmp_path):
        capture = tmp_path / "captured.jobs"
        fake_bin = self._fake_dynawo(tmp_path, capture)

        benchmark_solvers.run_single_benchmark(
            fake_bin, str(case_dir), IDA_CONFIG, str(tmp_path / "out"))

        leftovers = [f for f in os.listdir(case_dir)
                     if f.endswith(".jobs") and f != "Case.jobs"]
        assert leftovers == []
