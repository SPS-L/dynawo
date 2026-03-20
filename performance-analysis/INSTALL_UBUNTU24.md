# Building the Performance Analysis Branch on Ubuntu 24.04 LTS

Step-by-step instructions for cloning, building, and running the
[`performance-analysis-framework`](https://github.com/SPS-L/dynawo/tree/performance-analysis-framework)
branch of the [SPS-L/dynawo](https://github.com/SPS-L/dynawo) fork on
Ubuntu 24.04 LTS (Noble Numbat).

This branch adds a zero-overhead solver profiler, Python analysis tools, and
Nordic-system benchmarks on top of the upstream
[dynawo/dynawo](https://github.com/dynawo/dynawo) codebase.
The standard Dynawo build system (`myEnvDynawo.sh`) is used throughout.

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installing Dependencies](#installing-dependencies)
3. [Clone and Checkout](#clone-and-checkout)
4. [Create myEnvDynawo.sh](#create-myenvdynawosh)
5. [Build](#build)
6. [Verify the Build](#verify-the-build)
7. [Build with Profiling Enabled](#build-with-profiling-enabled)
8. [Run a Profiled Simulation](#run-a-profiled-simulation)
9. [Set Up Python Analysis Tools](#set-up-python-analysis-tools)
10. [Run Benchmarks](#run-benchmarks)
11. [Troubleshooting](#troubleshooting)

---

## System Requirements

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| RAM      | 8 GB    | 16 GB or more |
| Disk     | 20 GB free | 40 GB free |
| CPU      | 4 cores | 8+ cores |
| OS       | Ubuntu 24.04 LTS (Noble Numbat), 64-bit | |

The first full build (third-party libraries + Dynawo) is CPU- and
memory-intensive.  With `DYNAWO_NB_PROCESSORS_USED` set to the number of
cores, each parallel job can use up to 2 GB of RAM.  If you run out of
memory, lower this value.

---

## Installing Dependencies

Ubuntu 24.04 ships GCC 13, CMake 3.28+, Python 3.12, and OpenJDK 21 — all
of which satisfy Dynawo's requirements.

Install everything in one command:

```bash
sudo apt update && sudo apt install -y \
    git gcc g++ gfortran autoconf pkgconf automake make libtool cmake hwloc \
    openjdk-21-jdk \
    libblas-dev liblpsolve55-dev libarchive-dev doxygen doxygen-latex \
    liblapack-dev libexpat1-dev libsqlite3-dev zlib1g-dev gettext patch clang \
    python3 python3-pip python3-venv python3-lxml python3-psutil \
    libncurses-dev libreadline-dev libdigest-perl-md5-perl unzip gcovr lcov \
    libboost-all-dev lsb-release libxml2-utils wget libcurl4-openssl-dev rsync \
    libopenblas-openmp-dev qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
```

Set `JAVA_HOME` (add to `~/.bashrc` for persistence):

```bash
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
```

> **Note on Python:** Ubuntu 24.04 does not provide a bare `python` command
> by default.  The `myEnvDynawo.sh` script created below sets
> `DYNAWO_PYTHON_COMMAND` to `python3` to handle this.

---

## Clone and Checkout

Clone the SPS-L fork and switch to the `performance-analysis-framework`
branch:

```bash
git clone https://github.com/SPS-L/dynawo.git
cd dynawo
git checkout performance-analysis-framework
```

Verify you are on the correct branch:

```bash
git branch
# * performance-analysis-framework
```

---

## Create myEnvDynawo.sh

Dynawo uses a wrapper script called `myEnvDynawo.sh` as the single entry
point for building, running simulations, and managing the environment.
Create it at the repository root:

```bash
cat > myEnvDynawo.sh << 'EOF'
#!/bin/bash
export DYNAWO_HOME=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export DYNAWO_SRC_OPENMODELICA=$DYNAWO_HOME/OpenModelica/Source
export DYNAWO_INSTALL_OPENMODELICA=$DYNAWO_HOME/OpenModelica/Install

export DYNAWO_LOCALE=en_GB
export DYNAWO_RESULTS_SHOW=false
export DYNAWO_BROWSER=firefox

export DYNAWO_PYTHON_COMMAND=python3
export DYNAWO_NB_PROCESSORS_USED=$(nproc)

export DYNAWO_BUILD_TYPE=Release

$DYNAWO_HOME/util/envDynawo.sh $@
EOF
chmod +x myEnvDynawo.sh
```

Key variables you may want to change:

| Variable | Description | Default |
|----------|-------------|---------|
| `DYNAWO_NB_PROCESSORS_USED` | Parallel build jobs. Lower if you run out of RAM. | `$(nproc)` |
| `DYNAWO_BUILD_TYPE` | `Release`, `Debug`, or `RelWithDebInfo` | `Release` |
| `DYNAWO_PYTHON_COMMAND` | Python interpreter. Must be Python 3 on Ubuntu 24.04. | `python3` |
| `DYNAWO_RESULTS_SHOW` | Open result plots in browser after simulation. | `false` |

---

## Build

Build the third-party dependencies and Dynawo in one step:

```bash
./myEnvDynawo.sh build-user
```

This command:

1. Downloads, patches, and compiles all third-party libraries (SuiteSparse,
   SUNDIALS, Adept, OpenModelica, etc.) into `build/3rdParty/` and
   `install/3rdParty/`.
2. Configures and compiles Dynawo itself into `build/` and `install/`.
3. Compiles the preassembled Modelica models and solver descriptions.

A first build typically takes **30–90 minutes** on an 8-core machine
(most of the time is spent on third-party libraries; subsequent rebuilds of
Dynawo alone are much faster).

> **Tip:** If you only need to rebuild Dynawo after making C++ changes
> (e.g. to the solver profiler), use the faster:
> ```bash
> ./myEnvDynawo.sh build-dynawo
> ```

---

## Verify the Build

Run the Nordic DynaWaltz test case to confirm everything compiled correctly.
This is a 74-bus, 20-generator, 175-second long-term voltage stability
simulation:

```bash
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
```

The simulation should complete without errors.  Output files (curves,
timelines, final state) are written to a directory under `Nordic/outputs/`.

You can also run with automatic curve plotting:

```bash
./myEnvDynawo.sh jobs-with-curves examples/DynaWaltz/Nordic/Nordic.jobs
```

---

## Build with Profiling Enabled

The performance profiling instrumentation is controlled by the CMake option
`DYNAWO_PROFILING`.  When set to `ON`, all `DYN_PROFILE_PHASE` macros in the
solver code compile into active instrumentation.  When `OFF` (the default),
they compile to no-ops with zero overhead.

To enable profiling, do a clean build with the profiling flag.  You do
**not** need to rebuild the third-party libraries.

### Clean Profiling Build

Add the following to your `myEnvDynawo.sh` before the
`$DYNAWO_HOME/util/envDynawo.sh` call:

```bash
# RelWithDebInfo: optimised build with debug symbols (compatible with perf, Valgrind)
export DYNAWO_RELEASE_WITH_DEBUG=true

# Enable solver profiling instrumentation at compile time
export DYNAWO_CMAKE_OPTIONAL="-DDYNAWO_PROFILING=ON"
```

Then do a clean rebuild:

```bash
./myEnvDynawo.sh clean-build-dynawo
```

### Verify Profiling Is Active

After rebuilding, confirm that the profiling symbols are present:

```bash
# Check that the profiler singleton is compiled in
nm $(find install/ -name "libdynawo_SolversCommon*" | head -1) 2>/dev/null \
    | grep -i "SolverProfiler" | head -3
```

You should see symbols containing `SolverProfiler`.  If the output is empty,
the build did not pick up `DYNAWO_PROFILING=ON`.

---

## Run a Profiled Simulation

Set the runtime environment variables and run:

```bash
# Enable the profiler at runtime
export DYNAWO_PROFILING=1

# Where to write the profiling data (CSV or JSON based on extension)
mkdir -p results/nordic
export DYNAWO_PROFILE_OUTPUT=$(pwd)/results/nordic/profile.csv

# Run the Nordic test case
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
```

The profiler will:

1. Print a summary table to stdout showing per-phase timing.
2. Write detailed data to the CSV (or JSON) file specified by
   `DYNAWO_PROFILE_OUTPUT`.

Example output:

```
===============================================================
  DYNAWO SOLVER PERFORMANCE PROFILE
===============================================================
Phase                  Total(s)   Calls   Avg(ms)   Min(ms)   Max(ms)  Pct(%)
---------------------------------------------------------------
SimulationLoop           12.345       1  12345.00  12345.00  12345.00   100.0
SolverSolve              11.890       1  11890.00  11890.00  11890.00    96.3
JacobianEval              4.567     850      5.37      2.12     15.89    37.0
LinearSolve               3.210     850      3.78      1.56     12.34    26.0
ResidualEval              2.345    3200      0.73      0.41      3.21    19.0
...
===============================================================
```

---

## Set Up Python Analysis Tools

The Python scripts in `performance-analysis/` parse the CSV/JSON output and
generate charts, HTML comparison reports, and bottleneck summaries.

```bash
cd performance-analysis

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Available tools:

| Script | Purpose |
|--------|---------|
| `analyze_profile.py` | Parse CSV, generate phase-breakdown charts and statistics |
| `compare_runs.py` | Side-by-side comparison of two profiling runs with HTML report |
| `benchmark_solvers.py` | Automated multi-configuration benchmarking |
| `bottleneck_detector.py` | Automatic bottleneck identification and recommendations |
| `memory_analyzer.py` | Memory growth analysis and leak detection |

---

## Run Benchmarks

### Nordic SIM vs IDA Comparison

Compare the default SIM (fixed-step) solver with the IDA (variable-step)
solver on the same Nordic system:

```bash
export DYNAWO_PROFILING=1
mkdir -p results/nordic

# SIM solver (default Nordic configuration)
export DYNAWO_PROFILE_OUTPUT=$(pwd)/results/nordic/profile_sim.csv
./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs

# IDA solver (alternative configuration)
export DYNAWO_PROFILE_OUTPUT=$(pwd)/results/nordic/profile_ida.csv
./myEnvDynawo.sh jobs performance-analysis/benchmarks/Nordic_IDA.jobs
```

### Solver Configuration Variants

Predefined solver parameter files are available in
`performance-analysis/benchmarks/solver_configs/`:

| File | Description |
|------|-------------|
| `sim_nordic_baseline.par` | Baseline SIM config (matches the default) |
| `sim_nordic_fast.par` | Looser tolerances — faster but less accurate |
| `sim_nordic_accurate.par` | Tighter tolerances — more accurate but slower |
| `ida_nordic.par` | IDA solver configuration for Nordic |

### Multiple Runs for Statistical Significance

```bash
for i in $(seq 1 5); do
    export DYNAWO_PROFILE_OUTPUT=results/nordic/run_${i}.csv
    ./myEnvDynawo.sh jobs examples/DynaWaltz/Nordic/Nordic.jobs
done
```

---

## Troubleshooting

### `python: command not found` During Build

Ubuntu 24.04 does not provide a `python` symlink.  Make sure your
`myEnvDynawo.sh` includes:

```bash
export DYNAWO_PYTHON_COMMAND=python3
```

### Out of Memory During Build

Each parallel compilation job can use up to 2 GB of RAM.  Lower the
processor count in `myEnvDynawo.sh`:

```bash
export DYNAWO_NB_PROCESSORS_USED=4   # or 2
```

### Java Not Found

Ubuntu 24.04 ships OpenJDK 21.  Set `JAVA_HOME`:

```bash
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
```

If multiple Java versions are installed:

```bash
sudo update-alternatives --config java
```

### GCC ABI Mismatches / Linker Errors

If you see undefined-symbol errors when linking against system Boost:

```bash
# Add to myEnvDynawo.sh:
export DYNAWO_FORCE_CXX11_ABI=true
```

Then do a clean rebuild: `./myEnvDynawo.sh clean-build-all`

### Qt Not Found

Ubuntu 24.04 provides both Qt 5 and Qt 6.  Dynawo uses Qt 5 (for the model
launcher).  Ensure you have:

```bash
sudo apt install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
```

### Profiling Data Not Exported

1. Ensure both the build-time flag (`-DDYNAWO_PROFILING=ON`) and the
   runtime variable (`export DYNAWO_PROFILING=1`) are set.
2. Ensure the output directory exists: `mkdir -p $(dirname $DYNAWO_PROFILE_OUTPUT)`
3. The profiler writes data in its destructor at process exit.  A `kill -9`
   will prevent export — use `Ctrl+C` or let the simulation complete
   normally.

### Build Fails After Upstream Sync

If the `performance-analysis-framework` branch has been rebased onto a newer
upstream master, do a clean rebuild of the third-party libraries:

```bash
./myEnvDynawo.sh clean-build-all
```

This takes longer but ensures all dependencies are consistent.

---

*Author: Petros Aristidou, Sustainable Power Systems Lab (SPS-L) — https://sps-lab.org | info@sps-lab.org*  
*Last edited: March 2025*
