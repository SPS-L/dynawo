# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dynawo is a hybrid C++/Modelica open-source simulation tool for power systems, focused on numerical stability studies. It enforces strict separation between modeling (Modelica/C++) and solving (pluggable numerical solvers). Version 1.8.0, licensed MPL-2.0.

## Branch Purpose: Performance Optimization (TRAISIM)

This branch (`3_performance-analysis-framework`) exists to make Dynawo fast enough for **real-time operator training simulation** on the 6,000+ bus French EHV-HV system. The target is **RTR ≥ 1.0** (real-time ratio: simulation must run at least as fast as wall-clock time) on the `PFR_20240605_N_NB_all_retained` test case using the **SolverSIM fixed-step** solver.

The work follows a data-driven methodology: profile first, identify bottlenecks, implement targeted optimizations, validate with regression testing. All profiling and analysis infrastructure lives in `performance-analysis/`.

### Optimization Roadmap (from `performance-analysis/OPTIMIZATION_ROADMAP.md`)

**Phase 0 — Quick Wins (15-25% expected, low risk):**
- **A7 Event Severity Classification** — highest priority. Minor events (tap changers, OELs, AVR limits) currently trigger unnecessary full Jacobian + KLU symbolic refactorization. The FixedTimeStep solver infrastructure already supports severity levels; only model-layer reclassification is needed. Expected 10-15% gain during event-heavy periods.
- **A1 Adaptive Factorization Control** — structure hash + nnz tracking to skip redundant symbolic factorizations (5-8%)
- **A2 Matrix Structure Change Tolerance** — configurable threshold for structural changes (3-5%)
- **A3 KLU Numerical-Only Refactorization** — explicit `klu_refactor` vs. `klu_factor` path (5-7%)

**Phase 1 — Medium Effort (cumulative 25-40%):** Cache-optimized matrix layout (P3), std::map audit, improved COLAMD ordering (A4)

**Phase 2 — Advanced (cumulative 40-70%):** PGO (P7), LTO (P8), OpenMP Jacobian (P1), OpenMP SubModel (P2)

**Phase 3 — Research:** Partial Jacobian updates (A5), GPU acceleration (P9), Schur complement decomposition (A6)


## Build Commands

The local entry point is `myEnvDynawo.sh` (gitignored, per-developer). It sets environment variables (`DYNAWO_HOME`, `DYNAWO_BUILD_TYPE`, `DYNAWO_PYTHON_COMMAND`, etc.) and delegates to `util/envDynawo.sh`. Always use `myEnvDynawo.sh` instead of calling `util/envDynawo.sh` directly — it ensures the environment is correctly configured.

```bash
# Full baseline build (dependencies + core + models)
./myEnvDynawo.sh build-user

# Targeted rebuild after C++ changes only
./myEnvDynawo.sh build-dynawo

# Build order from clean (expensive — avoid unless needed):
# build-3rd-party → config-dynawo → build-dynawo

# Run a simulation
./myEnvDynawo.sh jobs path/to/config.jobs

# All available commands
./myEnvDynawo.sh help
```

The current `myEnvDynawo.sh` has profiling enabled (`-DDYNAWO_PROFILING=ON`), debug symbols in Release (`DYNAWO_RELEASE_WITH_DEBUG=true`), and uses all available cores.

## Testing

```bash
# Unit tests (Google Test)
./myEnvDynawo.sh build-tests
./myEnvDynawo.sh build-tests-coverage

# Non-regression tests (full suite is slow — filter when possible)
./myEnvDynawo.sh nrt
./myEnvDynawo.sh nrt -p "regex_pattern"    # filter by regex
./myEnvDynawo.sh nrt -n "name_filter"      # filter by name
```

## Architecture

Core source is under `dynawo/sources/` with these component boundaries:

- **Simulation/** — Main simulation loop orchestration and lifecycle
- **Solvers/** — Numerical solvers (IDA variable-step, KINSOL fixed-step, algebraic solvers, DDM distributed solver) with shared profiling in `Common/`
- **Modeler/** and **Models/** — Model interfaces and implementations (C++ and Modelica)
- **ModelicaCompiler/** — Modelica `.mo` → shared library compilation pipeline
- **API/** — Public data interfaces (jobs, curves, parameters, timelines, constraints)
- **Common/** — Shared utilities: logging, error handling, profiling (`DYNSolverProfiler`)
- **Launcher/** — CLI entry points
- **RT/** — Real-time simulation support

Design principle: Models expose residuals/Jacobians/zero-crossings; Solvers are generic and pluggable; neither layer knows the other's internals.

## Code Conventions

- **C++11**, linted with cpplint (config in `CPPLINT.cfg`: 160-char line limit, `root=dynawo/sources`)
- **Naming**: classes use `DYN*` prefix, namespaces are `DYN::`, macros are `DYNAWO_*`
- **Files**: named `DYN*.cpp` / `DYN*.h`
- Profiling instrumentation must be behind `DYNAWO_PROFILING` compile flag (zero cost when disabled)
- Keep changes scoped to owning component; avoid cross-layer coupling

## Environment

- `myEnvDynawo.sh` handles all environment setup — do not set `DYNAWO_HOME` or other variables manually
- Build artifacts (`build/`, `install/`, `deploy/`) are multi-GB — use `clean-dynawo` not `clean-all` when possible
- 3rd-party and OpenModelica builds are very expensive; don't trigger unless dependencies changed

## Performance Analysis Framework

Profiling is already enabled in `myEnvDynawo.sh` (`-DDYNAWO_PROFILING=ON` + debug symbols). The `DYNSolverProfiler` is an RAII-based, zero-overhead (when disabled) profiling system with 17 hierarchical phases spanning Simulation, Solver, and KLU layers.

### Profiling Workflow
```bash
# Run a simulation with profiling output
export DYNAWO_PROFILE_OUTPUT=profile.csv   # or .json
./myEnvDynawo.sh jobs path/to/config.jobs

# Analyze results (Python venv in performance-analysis/venv/)
cd performance-analysis
python analyze_profile.py profile.csv --output-dir results/
python bottleneck_detector.py profile.csv
python compare_runs.py baseline.csv optimized.csv --output comparison.html
python memory_analyzer.py profile.csv --output-dir results/memory/
```

### Investigation Methodology (`performance-analysis/SPRINT_GUIDE.md`)
1. **Phase Z** — Zero-knowledge baseline: 5 runs, take median, compute diagnostic ratios, run bottleneck detector
2. **Phase I** — Independent verification with `perf record` + flame graphs
3. **Phase P/V** — Implement optimization, validate with 5-run median, numerical correctness gate (L2 norm < 1e-5)

### Key Documentation
| File | Purpose |
|------|---------|
| `performance-analysis/PROFILING_FRAMEWORK.md` | Profiler architecture, phase taxonomy, API, known issues |
| `performance-analysis/SPRINT_GUIDE.md` | Step-by-step investigation methodology |
| `performance-analysis/OPTIMIZATION_ROADMAP.md` | 16-item optimization plan with code analysis and phased rollout |
| `performance-analysis/README.md` | Overview, tool docs, adding new instrumentation points |
| `performance-analysis/INSTALL_UBUNTU24.md` | Build setup guide with troubleshooting |
| `performance-analysis/TRAISIM_future_plans.md` | Executive summary presentation (Marp slides) |

### Analysis Tools
| Script | Purpose |
|--------|---------|
| `analyze_profile.py` | Parse CSV/JSON profiles, generate pie/bar/timeseries charts |
| `bottleneck_detector.py` | Automated hotspot identification with severity-ranked findings |
| `compare_runs.py` | Baseline vs. optimized comparison with HTML report |
| `memory_analyzer.py` | Peak memory analysis and leak detection via regression |
| `benchmark_solvers.py` | Multi-configuration benchmarking (IDA vs SIM, parameter sweeps) |

### Test Cases for Benchmarking
- **Nordic** (74 buses) — fast iteration during development
- **RTNordic** — medium-scale validation
- **PFR_20240605_N_NB** — large RTE EHV-HV network
- **PFR_20240605_N_NB_all_retained** — primary real-time target (6000+ buses, 4000s sim, fixed-step SIM)
- **PFR_20240605_events** — event-heavy stress test

## Key Entry Points

| File | Purpose |
|------|---------|
| `myEnvDynawo.sh` | Local entry point — sets env vars, delegates to `util/envDynawo.sh` (gitignored, per-developer) |
| `util/envDynawo.sh` | Underlying build/test/deploy wrapper (~3000 lines, all commands) |
| `dynawo/CMakeLists.txt` | Root CMake config |
| `dynawo/sources/Simulation/DYNSimulation.cpp` | Main simulation loop |
| `dynawo/sources/Solvers/Common/DYNSolverProfiler.h` | Profiler singleton + PhaseTimer RAII |
| `dynawo/sources/Solvers/FixedTimeStep/` | SolverSIM/TRAP — the target solver for TRAISIM |
| `dynawo/sources/Solvers/Common/` | Shared solver infrastructure (profiling, KLU interface) |
