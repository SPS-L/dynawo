# Dynawo Installation Guide for Ubuntu 24.04 LTS

This guide provides complete instructions for building and installing Dynawo from source on Ubuntu 24.04 LTS (Noble Numbat), including the performance profiling framework.

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installing Dependencies](#installing-dependencies)
3. [Building from Source](#building-from-source)
4. [Profiling Build Instructions](#profiling-build-instructions)
5. [Running Examples](#running-examples)
6. [Troubleshooting](#troubleshooting)

---

## System Requirements

### Hardware

| Resource | Minimum | Recommended |
|----------|---------|-------------|
| RAM      | 8 GB    | 16 GB or more |
| Disk     | 20 GB free | 40 GB free (includes 3rd-party builds and model compilation) |
| CPU      | 4 cores | 8+ cores (parallel builds scale well) |

The build process is CPU- and memory-intensive. Parallel compilation (`make -jN`) with N equal to the number of cores will use roughly 2 GB of RAM per thread during the heaviest compilation units. If you run out of memory, reduce the parallelism level.

### Operating System

- Ubuntu 24.04 LTS (Noble Numbat), 64-bit
- Kernel 6.x (the stock Ubuntu 24.04 kernel is supported)

---

## Installing Dependencies

### Build Essentials and Compilers

Ubuntu 24.04 ships with GCC 13 by default and GCC 14 available in the repositories. Either version works.

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    gcc-13 g++-13 \
    cmake \
    make
```

If you prefer GCC 14:

```bash
sudo apt install -y gcc-14 g++-14
```

To set GCC 13 or 14 as the default compiler:

```bash
# For GCC 13 (default on Ubuntu 24.04):
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 130 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13

# Or for GCC 14:
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 140 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-14

# Select which one to use:
sudo update-alternatives --config gcc
```

### Java Development Kit

Dynawo requires Java for the Modelica compiler toolchain. Ubuntu 24.04 provides OpenJDK 21 as the primary JDK.

```bash
sudo apt install -y openjdk-21-jdk
```

Verify the installation:

```bash
java -version
# Expected: openjdk version "21.x.x"
```

Set `JAVA_HOME` if it is not already configured:

```bash
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
```

Add this line to your `~/.bashrc` or `~/.profile` to make it persistent.

### Python

Dynawo build scripts and the performance analysis tools require Python 3. Ubuntu 24.04 no longer ships Python 2, so only Python 3 packages are needed.

```bash
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv
```

### Qt 6

Ubuntu 24.04 has moved to Qt 6 as the primary Qt version. The `qt6-base-dev` package replaces the older `qtbase5-dev`.

```bash
sudo apt install -y qt6-base-dev
```

### Boost Libraries

```bash
sudo apt install -y libboost-all-dev
```

This installs all Boost components. Dynawo uses at least Boost.Filesystem, Boost.Log, Boost.Program_options, Boost.Serialization, and Boost.System.

### XML and Compression Libraries

```bash
sudo apt install -y \
    libxml2-dev \
    zlib1g-dev \
    libarchive-dev
```

### Other Development Tools

```bash
sudo apt install -y \
    git \
    doxygen \
    graphviz \
    pkg-config \
    autoconf \
    automake \
    libtool \
    curl
```

### All Dependencies in a Single Command

For convenience, here is a single command that installs everything:

```bash
sudo apt update && sudo apt install -y \
    build-essential gcc-13 g++-13 cmake make \
    openjdk-21-jdk \
    python3 python3-pip python3-venv \
    qt6-base-dev \
    libboost-all-dev libxml2-dev zlib1g-dev libarchive-dev \
    git doxygen graphviz pkg-config autoconf automake libtool curl
```

---

## Building from Source

### Step 1: Clone the Repository

```bash
git clone https://github.com/dynawo/dynawo.git
cd dynawo
```

### Step 2: Configure with CMake

Create a build directory and run CMake:

```bash
mkdir -p build && cd build

cmake ../dynawo \
    -DCMAKE_INSTALL_PREFIX=$HOME/dynawo-install \
    -DCMAKE_BUILD_TYPE=Release \
    -DDYNAWO_HOME=$(pwd)/.. \
    -DINSTALL_OPENMODELICA=$HOME/OpenModelica \
    -DCMAKE_C_COMPILER=gcc-13 \
    -DCMAKE_CXX_COMPILER=g++-13
```

Key CMake variables:

| Variable | Description |
|----------|-------------|
| `CMAKE_INSTALL_PREFIX` | Where Dynawo will be installed |
| `CMAKE_BUILD_TYPE` | `Release`, `Debug`, `RelWithDebInfo`, or `MinSizeRel` |
| `DYNAWO_HOME` | Root of the Dynawo source tree |
| `INSTALL_OPENMODELICA` | Path to OpenModelica installation |
| `BUILD_TESTS` | Set to `ON` to build unit tests |
| `FORCE_CXX11_ABI` | Set to `ON` if you need the C++11 ABI (for compatibility with certain libraries) |

### Step 3: Build

```bash
# Use -j with the number of CPU cores for parallel builds
make -j$(nproc)
```

A full build from source typically takes 20-45 minutes on an 8-core machine, depending on CPU speed and available RAM.

### Step 4: Install

```bash
make install
```

The installation places binaries, libraries, shared data, and model databases under the path specified by `CMAKE_INSTALL_PREFIX`.

### Step 5: Set Up Environment

After installation, configure your environment:

```bash
export DYNAWO_HOME=$HOME/dynawo-install
export PATH=$DYNAWO_HOME/bin:$PATH
export LD_LIBRARY_PATH=$DYNAWO_HOME/lib:$LD_LIBRARY_PATH
```

Add these lines to `~/.bashrc` for persistence.

---

## Profiling Build Instructions

The Dynawo performance profiling framework is controlled by a CMake option and runtime environment variables. The profiler instruments solver phases (Jacobian evaluation, residual evaluation, linear solve, etc.) and exports timing data in CSV or JSON format.

### Build-Time Activation

Add the `-DDYNAWO_PROFILING=ON` flag to your CMake configuration:

```bash
cmake ../dynawo \
    -DCMAKE_INSTALL_PREFIX=$HOME/dynawo-install \
    -DCMAKE_BUILD_TYPE=Release \
    -DDYNAWO_HOME=$(pwd)/.. \
    -DINSTALL_OPENMODELICA=$HOME/OpenModelica \
    -DDYNAWO_PROFILING=ON
```

When `DYNAWO_PROFILING=ON` is set at build time, the profiling macros (`DYN_PROFILE_PHASE`, `DYN_PROFILE_PHASE_MEM`, etc.) compile into actual instrumentation code. Without this flag, they are compiled as no-ops with zero overhead.

### Runtime Activation

Even without the build-time flag, the runtime profiler singleton can be activated by setting an environment variable:

```bash
export DYNAWO_PROFILING=1
```

This enables the `SolverProfiler` singleton at process startup. Note that the RAII `PhaseTimer` macros will only be active if the build-time flag was also set. The runtime variable primarily controls whether manually instrumented `record()` calls are active.

### Automatic Data Export

To automatically export profiling data when the simulation finishes, set the `DYNAWO_PROFILE_OUTPUT` environment variable:

```bash
# Export as CSV
export DYNAWO_PROFILE_OUTPUT=/path/to/profile_output.csv

# Export as JSON (filename must end in .json)
export DYNAWO_PROFILE_OUTPUT=/path/to/profile_output.json
```

The profiler destructor checks this variable and writes the collected data before the process exits.

### Combined Example

```bash
# Configure with profiling
cd build
cmake ../dynawo \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DDYNAWO_PROFILING=ON \
    -DDYNAWO_HOME=$(pwd)/.. \
    -DCMAKE_INSTALL_PREFIX=$HOME/dynawo-install \
    -DINSTALL_OPENMODELICA=$HOME/OpenModelica

# Build
make -j$(nproc)
make install

# Run with profiling and auto-export
export DYNAWO_PROFILING=1
export DYNAWO_PROFILE_OUTPUT=$HOME/results/ieee14_profile.csv
$DYNAWO_HOME/bin/dynawo jobs --input examples/IEEE14/IEEE14.jobs
```

---

## Running Examples

After building and installing, test your installation with one of the bundled examples:

```bash
# Run the IEEE 14-bus example
cd $DYNAWO_HOME
dynawo jobs --input examples/IEEE14/IEEE14.jobs

# Run with verbose output
dynawo jobs --input examples/IEEE14/IEEE14.jobs --log-level DEBUG
```

Output files (curves, timelines, final state) are written to the output directory specified in the `.jobs` file.

### Running with Profiling

```bash
export DYNAWO_PROFILING=1
export DYNAWO_PROFILE_OUTPUT=$HOME/results/ieee14_profile.json

dynawo jobs --input examples/IEEE14/IEEE14.jobs

# The profiler will print a summary table to stdout and write detailed
# data to the JSON file specified above.
```

---

## Troubleshooting

### GCC Version Mismatches

**Symptom:** Linker errors about undefined symbols or ABI incompatibilities.

**Cause:** Mixing object files compiled with different GCC versions, or third-party libraries built with a different GCC than the main project.

**Solution:**
1. Ensure a consistent compiler version across the entire build. Set `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER` explicitly:
   ```bash
   cmake .. -DCMAKE_C_COMPILER=gcc-13 -DCMAKE_CXX_COMPILER=g++-13
   ```
2. If you previously built with a different GCC version, perform a clean rebuild:
   ```bash
   rm -rf build/*
   ```
3. If using system-installed Boost or other libraries built with a different ABI, try:
   ```bash
   cmake .. -DFORCE_CXX11_ABI=ON
   ```

### Missing Libraries at Runtime

**Symptom:** `error while loading shared libraries: libdynawo_*.so: cannot open shared object file`

**Solution:** Ensure `LD_LIBRARY_PATH` includes the Dynawo library directory:
```bash
export LD_LIBRARY_PATH=$DYNAWO_HOME/lib:$LD_LIBRARY_PATH
```

Alternatively, add the library path to the system linker cache:
```bash
echo "$DYNAWO_HOME/lib" | sudo tee /etc/ld.so.conf.d/dynawo.conf
sudo ldconfig
```

### Java Path Issues

**Symptom:** CMake cannot find Java, or the Modelica compiler fails with `JAVA_HOME is not set`.

**Solution:**
1. Verify Java is installed:
   ```bash
   java -version
   ```
2. Set `JAVA_HOME` explicitly:
   ```bash
   export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
   ```
3. Verify the path exists:
   ```bash
   ls $JAVA_HOME/bin/java
   ```
4. On Ubuntu 24.04, if multiple Java versions are installed, select the correct one:
   ```bash
   sudo update-alternatives --config java
   ```

### CMake Version Too Old

**Symptom:** `CMake Error at CMakeLists.txt: cmake_minimum_required VERSION 3.9.6`

**Solution:** Ubuntu 24.04 ships CMake 3.28+, which exceeds the minimum requirement. If you see this error, you may be running a containerized or minimal environment. Install CMake:
```bash
sudo apt install -y cmake
cmake --version
```

### Qt6 Not Found

**Symptom:** CMake reports it cannot find Qt or `QT_DIR` is not set.

**Solution:** Ubuntu 24.04 uses Qt 6. Ensure you installed `qt6-base-dev`, not the older Qt 5 packages:
```bash
sudo apt install -y qt6-base-dev
```

If CMake still cannot find Qt 6, you may need to set the prefix path:
```bash
cmake .. -DCMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/Qt6
```

### Boost Not Found or Wrong Version

**Symptom:** CMake reports `Could NOT find Boost` or links against the wrong version.

**Solution:**
```bash
sudo apt install -y libboost-all-dev
```

If you have a custom Boost installation, point CMake to it:
```bash
cmake .. -DBOOST_ROOT=/path/to/boost
```

### Out of Memory During Build

**Symptom:** The compiler is killed by the OOM killer, or you see `g++: fatal error: Killed signal terminated program`.

**Solution:** Reduce the parallelism level:
```bash
# Instead of make -j$(nproc), use fewer jobs:
make -j4   # or even make -j2
```

Each parallel compilation job can use 1.5-2 GB of RAM for the largest translation units. On a 16 GB system, limit to `-j6` or fewer.

### Profiling Data Not Exported

**Symptom:** You set `DYNAWO_PROFILE_OUTPUT` but no file is created.

**Solution:**
1. Ensure `DYNAWO_PROFILING=1` is set in your environment.
2. Ensure the build was configured with `-DDYNAWO_PROFILING=ON`.
3. Verify the output directory exists and is writable:
   ```bash
   mkdir -p $(dirname $DYNAWO_PROFILE_OUTPUT)
   ```
4. Check that the simulation actually ran to completion. The profiler exports data in its destructor, so a crash or forced kill (`kill -9`) will prevent export.
