# Dynawo Project Guidelines

## Build And Test
- Use the Dynawo wrapper entrypoint for almost all operations: `util/envDynawo.sh <command>`.
- Preferred local baseline build command: `util/envDynawo.sh build-user`.
- For targeted rebuilds after C++ changes, prefer `util/envDynawo.sh build-dynawo`.
- For non-regression tests, use `util/envDynawo.sh nrt` and filter when possible (`-p <regex>` or `-n <name_filter>`) because full NRT runs can be long.
- For unit tests, use `util/envDynawo.sh build-tests` (or `build-tests-coverage` when needed).
- Respect build order when invoking granular commands: `build-3rd-party` -> `config-dynawo` -> `build-dynawo`.

## Architecture
- Core source tree is under `dynawo/sources/`.
- Key component boundaries:
  - `Simulation/`: simulation loop orchestration and lifecycle.
  - `Solvers/`: numerical solvers and linear algebra coupling.
  - `Modeler/` and `Models/`: model interfaces and implementations.
  - `ModelicaCompiler/`: Modelica to shared-library compilation flow.
  - `API/`: public interfaces exposed to other components.
  - `Test/`: test framework and related targets.
- Keep changes scoped to the owning component whenever possible; avoid cross-layer coupling unless required by design.

## Conventions
- Follow existing C++ lint/style configuration in `CPPLINT.cfg` (notably 160-char line length and configured filters).
- Reuse existing CMake targets and wrapper commands before introducing new build scripts.
- Prefer small, incremental edits in `dynawo/sources/**` and preserve existing naming/layout patterns in each module.
- For profiling-related work, keep instrumentation behind compile-time flags (`DYNAWO_PROFILING`) and avoid runtime cost when disabled.

## Environment And Pitfalls
- Ensure `DYNAWO_HOME` points to repository root before running Dynawo tooling.
- On modern Ubuntu (for example 24.04), set `DYNAWO_PYTHON_COMMAND=python3` if `python` is unavailable.
- Build artifacts are large (`build/`, `install/`, `deploy/`); avoid unnecessary clean rebuilds.
- OpenModelica and 3rd-party dependency builds are expensive; do not trigger them unless required.

## Documentation Links
- Primary project entrypoint and official build/install docs: `README.md`.
- Full command catalog and behavior: `util/envDynawo.sh` (`help` output).
- CI command examples and expected automation patterns: `.github/workflows/ci.yml`.
- Profiling branch details and framework internals: `performance-analysis/PROFILING_FRAMEWORK.md`.
- Ubuntu 24.04 setup and troubleshooting for profiling work: `performance-analysis/INSTALL_UBUNTU24.md`.
- Performance tooling usage and analysis scripts: `performance-analysis/README.md`.

## Agent Behavior In This Workspace
- Prefer validating changes with the narrowest relevant command first (targeted build/test) before broader runs.
- When touching performance-analysis code or docs, align with the workflow and outputs documented under `performance-analysis/` instead of redefining process in-line.
- When uncertain about command choice, run `util/envDynawo.sh help` and use an existing command instead of inventing a new workflow.
