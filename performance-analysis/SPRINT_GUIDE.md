# Dynaωo Performance Sprint — Zero-Knowledge Investigation Framework

## Overview

This document is a self-contained sprint guide covering environment setup, full build, test-case deployment, and a four-phase data-driven investigation methodology. No assumption about which component is slow is accepted until profiler and `perf` data on the actual target system confirms it.

**Repositories**

| Repository | Branch | Purpose |
|---|---|---|
| `github.com/SPS-L/dynawo` | `3_performance-analysis-framework` | Dynawo core + profiling instrumentation |
| `github.com/SPS-L/TRAISIM` | `performance-analysis` | Dynawo-RTE models, test cases, Python tools |

**Sprint phases**

1. **Environment** — build, deploy, verify
2. **Phase Z** — zero-knowledge baseline collection
3. **Phase I** — independent hotspot verification
4. **Phase P/V** — prioritised implementation and validated regression

---

## Environment Setup

> For full system requirements, dependency installation, and troubleshooting see [`INSTALL_UBUNTU24.md`](https://github.com/SPS-L/dynawo/blob/3_performance-analysis-framework/performance-analysis/INSTALL_UBUNTU24.md) and [`TRAISIM/Readme.md §§1–9`](https://github.com/SPS-L/TRAISIM/blob/performance-analysis/Readme.md).

### Build Dynawo with Profiling

```bash
git clone git@github.com:SPS-L/dynawo.git
cd dynawo && git checkout 3_performance-analysis-framework

cat > myEnvDynawo.sh << 'EOF'
#!/bin/bash
export DYNAWO_HOME=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export DYNAWO_SRC_OPENMODELICA=$DYNAWO_HOME/OpenModelica/Source
export DYNAWO_INSTALL_OPENMODELICA=$DYNAWO_HOME/OpenModelica/Install
export DYNAWO_LOCALE=en_GB
export DYNAWO_RESULTS_SHOW=false
export DYNAWO_PYTHON_COMMAND=python3
export DYNAWO_NB_PROCESSORS_USED=$(nproc)
export DYNAWO_BUILD_TYPE=Release
export DYNAWO_RELEASE_WITH_DEBUG=true
export DYNAWO_CMAKE_OPTIONAL="-DDYNAWO_PROFILING=ON -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer"
$DYNAWO_HOME/util/envDynawo.sh $@
EOF
chmod +x myEnvDynawo.sh

./myEnvDynawo.sh build-user 2>&1 | tee build.log
./myEnvDynawo.sh deploy
cd ..
```

> `DYNAWO_RELEASE_WITH_DEBUG=true` adds `-g` symbols while keeping `DYNAWO_BUILD_TYPE=Release`. Do not change the build type — it is used as a directory-name segment in the build tree and changing it breaks path alignment with Dynawo-RTE. `-fno-omit-frame-pointer` is required for `perf` call-graph reconstruction.

For subsequent rebuilds after code changes:
```bash
./myEnvDynawo.sh clean-build-dynawo && ./myEnvDynawo.sh deploy
```

### Build Dynawo-RTE

`libiidm-rte` is hosted on RTE's internal GitLab (not public); the TRAISIM repo ships it as `libiidm-rte.zip`.

```bash
GCC_VER=$(gcc -dumpversion | cut -d. -f1)
THIRDPARTY_SRC="dynawo-rte/build/3rdParty/gcc${GCC_VER}/shared/Release/src"
mkdir -p "${THIRDPARTY_SRC}"
unzip -o libiidm-rte.zip -d "${THIRDPARTY_SRC}"

sed -i 's|GIT_REPOSITORY.*|DOWNLOAD_COMMAND ""|; /GIT_TAG/d; /GIT_PROGRESS/d' \
    dynawo-rte/dynawo/3rdParty/libiidm-rte/libiidm-rte.cmake
sed -i 's/-Werror //' "${THIRDPARTY_SRC}/libiidm-rte/CMakeLists.txt"
sed -i 's/\(filesystem\)/date_time \1/' dynawo-rte/dynawo/CMakeLists.txt

cd dynawo-rte
cat > myEnvDynawoRTE.sh << 'RTEOF'
#!/bin/bash
export DYNAWO_RTE_HOME=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TRAISIM_ROOT=$(cd "${DYNAWO_RTE_HOME}/.." && pwd)

export DYNAWO_HOME=$(find "${TRAISIM_ROOT}/dynawo/deploy" -type d -name "dynawo" | head -1)

if [ ! -d "$DYNAWO_HOME" ]; then
  echo "Dynawo deploy directory not found — run: cd dynawo && ./myEnvDynawo.sh deploy"
  exit 1
fi

export DYNAWO_RTE_LOCALE=en_GB
export DYNAWO_RESULTS_SHOW=false
[ -z "$DYNAWO_NB_PROCESSORS_USED" ] && export DYNAWO_NB_PROCESSORS_USED=$(nproc)
export DYNAWO_BUILD_TYPE=Release
export DYNAWO_PYTHON_COMMAND=python3

$DYNAWO_RTE_HOME/util/envDynawoRTE.sh $@
RTEOF
chmod +x myEnvDynawoRTE.sh

./myEnvDynawoRTE.sh build-all 2>&1 | tee build.log
cd ..
```

> Do not set `DYNAWO_LIBIIDM_RTE_HOME`. Leaving it unset lets `build-all` compile libiidm-rte with the same Boost instance used by Dynawo-RTE, avoiding imported-target linker errors.

Verify: `cd dynawo-rte && ./myEnvDynawoRTE.sh jobs ../testcases/Nordic/Nordic.jobs`

### Python Analysis Tools

```bash
cd dynawo/performance-analysis
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt
cd ../..
```

Enable `perf` call-graph capture (once, system-wide):

```bash
sudo sh -c 'echo 1 > /proc/sys/kernel/perf_event_paranoid'
sudo sh -c 'echo 0 > /proc/sys/kernel/kptr_restrict'
```

---

## Available Test Cases

| Case | Scale | Purpose |
|---|---|---|
| `Nordic/` | 74 buses | Fast iteration, sanity check |
| `RTNordic/` | Medium | Intermediate scale |
| `PFR_20240605_N_NB/` | Large RTE EHV-HV | Secondary target |
| `PFR_20240605_N_NB_all_retained/` | Large (all branches retained) | **Primary real-time target** |
| `PFR_20240605_N_NB_all_retained/PFR_20240605_N_NB_4000s.jobs` | Large + 8 scheduled events, 4000 s | Discrete-event stress |

The PFR cases use `dynawo_SolverSIM` (fixed-step), `stopTime="4000"`, `useStandardModels="true"`, and must be launched from their own directory via `myEnvDynawoRTE.sh`.

---

## Phase Z — Zero-Knowledge Baseline

### Z.1 — Five Runs, Take Median

```bash
for RUN in $(seq 1 5); do
    export DYNAWO_PROFILE_OUTPUT=$(pwd)/results/PFR/run_${RUN}.csv
    (cd testcases/PFR_20240605_N_NB_all_retained && \
     ../../dynawo-rte/myEnvDynawoRTE.sh jobs PFR_20240605_N_NB.jobs)
done
```

Select the median CSV by `SimulationLoop` `total_seconds`:

```python
import pandas as pd, shutil, os
from io import StringIO

runs = {}
for i in range(1, 6):
    path = f"results/PFR/run_{i}.csv"
    raw  = open(path).read().split("\n\n", 1)[0]
    df   = pd.read_csv(StringIO(raw), comment="#")
    df.columns = [c.strip().lower() for c in df.columns]
    t = float(df.loc[df["phase"].str.strip().str.lower() == "simulationloop", "total_seconds"].iloc[0])
    runs[i] = t
    print(f"  run_{i}.csv  SimulationLoop = {t:.3f} s")

median_run = sorted(runs, key=runs.get)[len(runs) // 2]
print(f"\nMedian run: run_{median_run}.csv ({runs[median_run]:.3f} s)")
shutil.copy(f"results/PFR/run_{median_run}.csv", "results/PFR/run_median.csv")
```

### Z.2 — Full Phase Table

Print every phase — no filtering. The dominant phase by time is the first investigation target, whatever it turns out to be:

```python
import pandas as pd
from io import StringIO

raw = open("results/PFR/run_median.csv").read().split("\n\n", 1)[0]
df  = pd.read_csv(StringIO(raw), comment="#")
df.columns = [c.strip().lower() for c in df.columns]
df  = df.set_index(df["phase"].str.lower())

total = float(df.loc["simulationloop", "total_seconds"])

print(f"{'Phase':30s}  {'%':>6}  {'Total(s)':>10}  {'Calls':>10}  {'Mean(ms)':>10}")
print("-" * 72)
for phase, row in df.iterrows():
    if phase == "simulationloop": continue
    t = float(row["total_seconds"])
    n = float(row["call_count"])
    print(f"  {phase:28s}  {t/total*100:6.1f}  {t:10.3f}  {int(n):10d}  {t/max(n,1)*1000:10.3f}")

print(f"\n  {'SimulationLoop':28s}  {'100.0':>6}  {total:10.3f}")
```

### Z.3 — Diagnostic Ratios

```python
def t(p): return float(df.loc[p,"total_seconds"]) if p in df.index else 0.0
def n(p): return float(df.loc[p,"call_count"])    if p in df.index else 0.0

print(f"jac_per_step     = {n('jacobianeval') / max(n('solverstep'),1):.3f}")
print(f"klu_sym_per_jac  = {n('klusymbolic')  / max(n('jacobianeval'),1):.3f}")
print(f"reinit_per_step  = {n('reinit')        / max(n('solverstep'),1):.4f}")
print(f"io_fraction      = {t('io')            / total:.3f}")
```

### Z.4 — Automated Bottleneck Detector

```bash
source dynawo/performance-analysis/venv/bin/activate
python3 dynawo/performance-analysis/bottleneck_detector.py \
    results/PFR/run_median.csv
```

Treat every `CRITICAL` / `WARNING` finding as a hypothesis to verify in Phase I.

### Z.5 — Timestep-Level Temporal Analysis

```bash
python3 dynawo/performance-analysis/analyze_profile.py \
    results/PFR/run_median.csv \
    --output-dir results/PFR/charts/baseline
```

Examine `step_duration_ts.png`:

- **Isolated spikes** at specific times → event-handling path
- **Sustained elevation after a point in time** → structural change at that moment (e.g. reinit cascade)
- **Monotonically increasing** → accumulating cost or memory pressure
- **Flat throughout** → steady-state solver cost dominates

### Z.6 — Memory Baseline

```bash
python3 dynawo/performance-analysis/memory_analyzer.py \
    results/PFR/run_median.csv \
    --output-dir results/PFR/memory/baseline
```

A non-zero linear slope indicates a leak. Fix before proceeding — it corrupts multi-sprint progress comparisons.

---

## Phase I — Independent Hotspot Verification

### I.1 — perf Record

```bash
cd testcases/PFR_20240605_N_NB_all_retained
perf record \
    -F 999 \
    --call-graph dwarf,32768 \
    -o ../../results/perf/pfr.data \
    -- \
    ../../dynawo-rte/myEnvDynawoRTE.sh jobs PFR_20240605_N_NB.jobs
cd ../..
```

### I.2 — Discover DSO Names, Then Filter to Dynawo

```bash
perf report \
    -i results/perf/pfr.data \
    --stdio --sort dso \
    --no-children --call-graph none \
    | grep -v '^#' | grep -v '^$' \
    | head -40 \
    | tee results/perf/dso_list.txt
```

> `--call-graph none` is required to get flat DSO output; without it, `perf report` produces a call-tree even with `--sort dso`.

Build the DSO filter from what was actually recorded, then re-run scoped to Dynawo libraries. Note that Dynawo DSOs have varied naming: `DYNModelNetwork.so`, `libdynawo_*.so`, `dynawo_*.so`, and the `dynawo` binary itself — use a case-insensitive grep:

```bash
DYNAWO_DSOS=$(grep -iE 'dynawo|DYN' results/perf/dso_list.txt \
    | awk '{print $NF}' | paste -sd ',' -)

perf report \
    -i results/perf/pfr.data \
    --stdio \
    --dsos "${DYNAWO_DSOS}" \
    --sort comm,dso,symbol \
    --no-children --call-graph none \
    | head -60 \
    | tee results/perf/top_symbols_dynawo.txt
```

> Use `--dsos` (plural) — the singular `--dso` is not a valid flag and is silently ignored.

If the dominant symbols match a Phase Z finding, the hypothesis is confirmed. If they do not, the discrepancy is itself informative — investigate it before continuing.

### I.3 — Flame Graph

```bash
git clone https://github.com/brendangregg/FlameGraph ~/FlameGraph

# Full flame graph (unfiltered — shows all DSOs in context)
perf script -i results/perf/pfr.data \
    | ~/FlameGraph/stackcollapse-perf.pl \
    | ~/FlameGraph/flamegraph.pl \
        --title "Dynaωo PFR — Full Profile" \
        --width 1800 \
    > results/perf/flamegraph_full.svg

# Dynawo-only flame graph (filter collapsed stacks to DYN:: frames)
perf script -i results/perf/pfr.data \
    | ~/FlameGraph/stackcollapse-perf.pl \
    | grep -i 'DYN::' \
    | ~/FlameGraph/flamegraph.pl \
        --title "Dynaωo PFR — Dynawo Frames Only" \
        --width 1800 \
    > results/perf/flamegraph_dynawo.svg
```

> Do not filter `perf script` output before `stackcollapse-perf.pl` — it expects complete stack traces. Filter the collapsed one-line-per-stack output instead, where grepping for `DYN::` selects stacks that pass through Dynawo code.

Any function occupying ≥ 2% of width is a valid investigation target.

### I.4 — Call-Origin Verification

For any expensive function identified above, confirm where it is being called from:

```bash
perf report \
    -i results/perf/pfr.data \
    --stdio --no-children \
    --call-graph graph,0.5 \
    --symbol-filter='<function_name>'
```

> Use `--symbol-filter` instead of piping through `grep -A` — it preserves the complete call-graph context for the matched symbol. Replace `<function_name>` with the symbol name (e.g., `ModelBus::evalF`); partial matches work.

The same function called from initialisation, from steady-state stepping, or from event handling requires a completely different response.

### I.5 — Violation-Filtered Profiling

The full-run profile is dominated by the many cheap steps (~72 ms each), drowning out the few expensive violation steps (~1000-1500 ms). Since violation steps collect far more samples at a fixed sampling rate, filtering to their time windows reveals the true hotspot structure.

**Step 1 — Find violation wall-clock windows from the log:**

```bash
grep "Algebraic mode (with J recalculation)" \
    testcases/PFR_20240605_N_NB_all_retained/outputs/logs/dynawo.log
```

Note the wall-clock timestamps (e.g., `2026-04-06 00:49:11` for t=10). Each violation spans ~1-2 seconds.

**Step 2 — Map to perf monotonic timestamps:**

```bash
perf script --header-only -i results/perf/pfr.data 2>/dev/null \
    | grep -E 'time of first sample|time of last sample'
```

Compute the offset: `perf_time = first_sample + (violation_wallclock - sim_start_wallclock)`.

**Step 3 — Extract violation-window samples:**

```bash
# Replace timestamps below with actual computed values
perf script -i results/perf/pfr.data \
    --time "<t10_start>,<t10_end> <t20_start>,<t20_end> <t300_start>,<t300_end>" \
    > /tmp/perf_violations.script
```

**Step 4 — Violation-only flame graph and top functions:**

```bash
# Flame graph
cat /tmp/perf_violations.script \
    | ~/FlameGraph/stackcollapse-perf.pl \
    > /tmp/violations_collapsed.txt

cat /tmp/violations_collapsed.txt \
    | ~/FlameGraph/flamegraph.pl \
        --title "PFR — Violation Steps Only" \
        --width 1800 \
    > results/perf/flamegraph_violations.svg

# Top leaf functions by self-time
TOTAL=$(awk '{sum += $NF} END {print sum}' /tmp/violations_collapsed.txt)

awk -F';' '{print $NF}' /tmp/violations_collapsed.txt \
    | awk '{count[$1]+=$2} END {for(f in count) print count[f], f}' \
    | sort -rn | head -20 | while read cnt func; do
        pct=$(echo "scale=2; $cnt * 100 / $TOTAL" | bc)
        printf "%6s  %6s%%  %s\n" "$cnt" "$pct" "$func"
    done
```

Compare the violation-only top functions against the full-run top functions (I.2). Functions that appear high in violations but not in the full run are the event-handling bottlenecks.

### I.6 — Investigation Checklist

Do not proceed to Phase P until every item has a measured answer:

```
□ What is the dominant phase by time?                              [Z.2 table]
□ What fraction of total is steady-state stepping vs. Reinit?     [Z.2 table]
□ What are the top-5 symbols by perf self-time (full run)?        [I.2]
□ Does the flame graph show unexpected width?                      [I.3]
□ For the dominant expensive function: what is its call origin?    [I.4]
□ What are the top-5 symbols during violation steps only?          [I.5]
□ Which functions appear in violations but not in the full run?    [I.5 vs I.2]
□ Does step_duration_ts show event-correlated spikes or flat?      [Z.5]
□ Is memory growing over the simulation?                           [Z.6]
□ Does the cost pattern repeat at Nordic scale and PFR scale?      [Z.1]
```

---

## Phase P — Prioritised Implementation

Phase P items are constructed bottom-up from Phase I findings. Items without a confirmed checklist entry are deferred.

| What Phase I found dominant | Candidate actions | Risk |
|---|---|---|
| Symbolic factorisation | Event severity classification; structure-hash reuse | Medium |
| Full numeric factorisation with no reuse | Enable numeric reuse path | Low |
| Residual evaluation, concentrated in one model type | Investigate that model's `evalF` | Low |
| Residual evaluation, spread uniformly | Vectorisation, caching | Medium |
| Unexpected hotspot (logging, containers, allocation) | Fix the specific function | Low |
| Memory leak | Fix — blocking before all other work | Blocking |
| I/O dominant | Reduce curve export frequency in `.crt` / `.jobs` | Zero |
| Flat step cost throughout | PGO, LTO, `-march=native` | Low |

---

## Phase V — Validated Regression Loop

### V.1 — Measurement Protocol

```bash
cp results/PFR/run_median.csv results/PFR/baseline_v${VERSION}.csv

./myEnvDynawo.sh clean-build-dynawo && ./myEnvDynawo.sh deploy
cd dynawo-rte && ./myEnvDynawoRTE.sh build-all && cd ..

for RUN in $(seq 1 5); do
    export DYNAWO_PROFILE_OUTPUT=$(pwd)/results/PFR/opt_run_${RUN}.csv
    (cd testcases/PFR_20240605_N_NB_all_retained && \
     ../../dynawo-rte/myEnvDynawoRTE.sh jobs PFR_20240605_N_NB.jobs)
done

python3 dynawo/performance-analysis/compare_runs.py \
    results/PFR/baseline_v${VERSION}.csv \
    results/PFR/opt_median.csv \
    --output results/comparisons/v${VERSION}_$(date +%Y%m%d_%H%M).html
```

### V.2 — Numerical Correctness Gate

```bash
python3 - << 'EOF'
import pandas as pd, numpy as np
ref = pd.read_csv("results/PFR/reference_curves/curves.csv")
opt = pd.read_csv("results/PFR/optimised_curves/curves.csv")
THRESHOLD = 1e-5
failed = []
for col in ref.columns[1:]:
    err = np.linalg.norm(ref[col] - opt[col]) / (np.linalg.norm(ref[col]) + 1e-12)
    if err > THRESHOLD: failed.append(col)
    print(f"{'FAIL' if err > THRESHOLD else 'OK  '}  {col}: {err:.2e}")
print(f"\n{'ALL OK' if not failed else f'FAILED: {len(failed)} curves'}")
EOF
```

### V.3 — Accept/Reject

| Result | Decision |
|---|---|
| Speedup ≥ expected, curves OK | **ACCEPT** — commit, update baseline |
| Speedup < expected but > 0, curves OK | **INVESTIGATE** — re-run `perf`; bottleneck may have shifted |
| Speedup = 0 or negative | **REJECT** — document finding, move to next item |
| Curves fail | **HARD REJECT** — do not re-attempt without root-cause analysis |

### V.4 — Real-Time Progress Tracking

```
RTR_k = 4000 / T_k     (T_sim = 4000 s, T_k = SimulationLoop median after change k)
Target: RTR ≥ 1.0
```

Sprint objective is met when RTR ≥ 1.0 on `PFR_20240605_N_NB_all_retained`.

---

*Repositories: [SPS-L/dynawo](https://github.com/SPS-L/dynawo/tree/3_performance-analysis-framework) · [SPS-L/TRAISIM](https://github.com/SPS-L/TRAISIM/tree/performance-analysis)*
