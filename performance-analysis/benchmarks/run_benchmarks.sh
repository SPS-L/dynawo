#!/bin/bash
# =============================================================================
# Dynawo Performance Benchmark Runner
# =============================================================================
# A comprehensive framework script for building Dynawo with profiling enabled,
# running benchmark simulations, collecting profiling data, and generating
# comparison reports.
#
# NOTE: This is a framework script that needs to be adapted to your specific
# environment. Paths, simulation cases, and analysis scripts should be
# configured to match your Dynawo installation and test cases.
#
# Make this file executable before running:
#   chmod +x run_benchmarks.sh
# =============================================================================

set -euo pipefail

# =============================================================================
# Configuration Variables
# =============================================================================
DYNAWO_HOME="${DYNAWO_HOME:-$(cd "$(dirname "$0")/../.." && pwd)}"
BUILD_DIR="${BUILD_DIR:-${DYNAWO_HOME}/build}"
RESULTS_DIR="${RESULTS_DIR:-${DYNAWO_HOME}/performance-analysis/benchmarks/results}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${RESULTS_DIR}/run_${TIMESTAMP}"
SKIP_BUILD=false
CMAKE_BUILD_TYPE="Release"
NUM_JOBS="$(nproc 2>/dev/null || echo 4)"

# Simulation cases to benchmark
# Each entry is a path to a .jobs file relative to DYNAWO_HOME
SIMULATION_CASES=(
    "examples/DynaWaltz/Nordic/Nordic.jobs"
    "examples/DynaWaltz/NordicTCB/NordicTCB.jobs"
)

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo "[INFO]  $(date '+%Y-%m-%d %H:%M:%S') - $*"
}

log_error() {
    echo "[ERROR] $(date '+%Y-%m-%d %H:%M:%S') - $*" >&2
}

check_prereqs() {
    local missing=()

    if ! command -v cmake &>/dev/null; then
        missing+=("cmake")
    fi

    if ! command -v python3 &>/dev/null; then
        missing+=("python3")
    fi

    if ! command -v make &>/dev/null && ! command -v ninja &>/dev/null; then
        missing+=("make or ninja")
    fi

    if [ ${#missing[@]} -gt 0 ]; then
        log_error "Missing prerequisites: ${missing[*]}"
        log_error "Please install the missing tools and try again."
        return 1
    fi

    log_info "All prerequisites satisfied."
    return 0
}

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Dynawo Performance Benchmark Runner

Options:
  --dynawo-home DIR    Path to Dynawo root directory (default: auto-detected)
  --build-dir DIR      Path to build directory (default: \${DYNAWO_HOME}/build)
  --output-dir DIR     Path to store benchmark results (default: \${DYNAWO_HOME}/performance-analysis/benchmarks/results)
  --skip-build         Skip the build step (use existing build)
  --jobs N             Number of parallel build jobs (default: $(nproc 2>/dev/null || echo 4))
  --cases CASE1,CASE2  Comma-separated list of simulation cases to run
  -h, --help           Show this help message

Examples:
  $(basename "$0") --dynawo-home /opt/dynawo --output-dir /tmp/benchmarks
  $(basename "$0") --skip-build --cases examples/DynaWaltz/Nordic/Nordic.jobs
  $(basename "$0") --jobs 8

Environment Variables:
  DYNAWO_HOME          Dynawo root directory
  BUILD_DIR            Build directory
  RESULTS_DIR          Results output directory

NOTE: This is a framework script. You may need to adapt paths and simulation
case configurations to match your specific Dynawo installation.
EOF
}

# =============================================================================
# Parse Command-Line Arguments
# =============================================================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dynawo-home)
            DYNAWO_HOME="$2"
            shift 2
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --output-dir)
            RESULTS_DIR="$2"
            RUN_DIR="${RESULTS_DIR}/run_${TIMESTAMP}"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --jobs)
            NUM_JOBS="$2"
            shift 2
            ;;
        --cases)
            IFS=',' read -ra SIMULATION_CASES <<< "$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# =============================================================================
# Main Execution
# =============================================================================

log_info "=============================================="
log_info "Dynawo Performance Benchmark Runner"
log_info "=============================================="
log_info "DYNAWO_HOME:   ${DYNAWO_HOME}"
log_info "BUILD_DIR:     ${BUILD_DIR}"
log_info "RESULTS_DIR:   ${RESULTS_DIR}"
log_info "RUN_DIR:       ${RUN_DIR}"
log_info "SKIP_BUILD:    ${SKIP_BUILD}"
log_info "NUM_JOBS:      ${NUM_JOBS}"
log_info "CASES:         ${SIMULATION_CASES[*]}"
log_info "=============================================="

# Check prerequisites
check_prereqs

# Create output directories
mkdir -p "${RUN_DIR}"
mkdir -p "${RUN_DIR}/profiling_data"
mkdir -p "${RUN_DIR}/reports"

# ---- Step 1: Build Dynawo with Profiling Enabled ----
if [ "${SKIP_BUILD}" = false ]; then
    log_info "Step 1: Building Dynawo with profiling enabled..."
    cd "${DYNAWO_HOME}"

    if [ -x "${DYNAWO_HOME}/myEnvDynawo.sh" ]; then
        ./myEnvDynawo.sh build-user 2>&1 | tee "${RUN_DIR}/build_output.log"
    else
        mkdir -p "${BUILD_DIR}"
        cd "${BUILD_DIR}"
        cmake "${DYNAWO_HOME}" \
            -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
            -DDYNAWO_PROFILING=ON \
            -DCMAKE_INSTALL_PREFIX="${BUILD_DIR}/install" \
            2>&1 | tee "${RUN_DIR}/cmake_output.log"
        make -j"${NUM_JOBS}" 2>&1 | tee "${RUN_DIR}/build_output.log"
    fi

    log_info "Step 1: Build complete."
else
    log_info "Step 1: Skipping build (--skip-build specified)."
fi

# ---- Step 2: Run Benchmark Simulations ----
log_info "Step 2: Running benchmark simulations..."

cd "${DYNAWO_HOME}"

for jobs_file in "${SIMULATION_CASES[@]}"; do
    # Derive a short case name from the jobs file path
    case_name="$(basename "${jobs_file}" .jobs)"
    log_info "  Running simulation case: ${case_name} (${jobs_file})"
    case_dir="${RUN_DIR}/profiling_data/${case_name}"
    mkdir -p "${case_dir}"

    if [ ! -f "${DYNAWO_HOME}/${jobs_file}" ]; then
        log_error "  Jobs file not found: ${DYNAWO_HOME}/${jobs_file} (skipping)"
        continue
    fi

    # Set up profiling output path (profiling is enabled at build time)
    export DYNAWO_PROFILE_OUTPUT="${case_dir}/profile.csv"

    # Run using myEnvDynawo.sh (the standard Dynawo CLI)
    if [ -x "${DYNAWO_HOME}/myEnvDynawo.sh" ]; then
        sim_start=$(date +%s.%N)
        ./myEnvDynawo.sh jobs "${jobs_file}" \
            2>&1 | tee "${case_dir}/simulation.log" || {
                log_error "    Simulation failed for ${case_name}"
                continue
            }
        sim_end=$(date +%s.%N)
        awk -v s="${sim_start}" -v e="${sim_end}" 'BEGIN{printf "%.2f\n", e - s}' \
            > "${case_dir}/wall_clock_seconds.txt"
    else
        log_error "    myEnvDynawo.sh not found at ${DYNAWO_HOME}"
        log_error "    Please ensure Dynawo is properly set up."
        continue
    fi
done

log_info "Step 2: Simulations complete."

# ---- Step 3: Collect Profiling CSV Data ----
log_info "Step 3: Collecting profiling data..."

profiling_summary="${RUN_DIR}/profiling_data/profiling_summary.csv"
echo "case,metric,value" > "${profiling_summary}"

for case_dir in "${RUN_DIR}"/profiling_data/*/; do
    case_name="$(basename "${case_dir}")"

    # Wall time measured around the simulation run in Step 2
    if [ -f "${case_dir}/wall_clock_seconds.txt" ]; then
        wall_time="$(cat "${case_dir}/wall_clock_seconds.txt")"
    else
        wall_time="N/A"
    fi
    echo "${case_name},wall_clock_seconds,${wall_time}" >> "${profiling_summary}"

    # Collect the CSV profiling output from Dynawo (the profiler's JSON
    # export has no analysis-tool consumer, so it is not collected)
    for data_file in "${case_dir}"/*.csv; do
        if [ -f "${data_file}" ]; then
            cp "${data_file}" "${RUN_DIR}/profiling_data/${case_name}_$(basename "${data_file}")"
        fi
    done
done

log_info "Step 3: Profiling data collected at ${profiling_summary}"

# ---- Step 4: Run Python Analysis Scripts ----
log_info "Step 4: Running Python analysis scripts..."

ANALYSIS_SCRIPTS_DIR="${DYNAWO_HOME}/performance-analysis"

for case_dir in "${RUN_DIR}"/profiling_data/*/; do
    case_name="$(basename "${case_dir}")"
    profile_csv="${case_dir}/profile.csv"

    if [ ! -f "${profile_csv}" ]; then
        log_info "  No profile.csv for ${case_name}, skipping analysis."
        continue
    fi

    report_dir="${RUN_DIR}/reports/${case_name}"
    mkdir -p "${report_dir}"

    # Visualize profile (charts + console summary)
    log_info "  Analyzing profile for ${case_name}..."
    python3 "${ANALYSIS_SCRIPTS_DIR}/analyze_profile.py" \
        "${profile_csv}" \
        --output-dir "${report_dir}" \
        2>&1 | tee "${report_dir}/analyze_profile.log" || {
            log_error "    analyze_profile.py failed for ${case_name}"
        }

    # Detect bottlenecks
    python3 "${ANALYSIS_SCRIPTS_DIR}/bottleneck_detector.py" \
        "${profile_csv}" \
        2>&1 | tee "${report_dir}/bottleneck_detector.log" || {
            log_error "    bottleneck_detector.py failed for ${case_name}"
        }

    # Memory analysis
    python3 "${ANALYSIS_SCRIPTS_DIR}/memory_analyzer.py" \
        "${profile_csv}" \
        --output-dir "${report_dir}" \
        2>&1 | tee "${report_dir}/memory_analyzer.log" || {
            log_error "    memory_analyzer.py failed for ${case_name}"
        }
done

log_info "Step 4: Analysis complete."

# ---- Step 5: Generate Comparison Reports ----
log_info "Step 5: Generating comparison reports..."

# Check if there are previous runs to compare against
previous_runs=()
for prev_dir in "${RESULTS_DIR}"/run_*/; do
    if [ -d "${prev_dir}" ] && [ "${prev_dir}" != "${RUN_DIR}/" ]; then
        previous_runs+=("${prev_dir}")
    fi
done

if [ ${#previous_runs[@]} -gt 0 ]; then
    log_info "  Found ${#previous_runs[@]} previous run(s) for comparison."

    # Take the most recent previous run
    latest_previous="${previous_runs[-1]}"
    log_info "  Comparing against: $(basename "${latest_previous}")"

    # Compare each case that has profile CSVs in both runs
    for case_dir in "${RUN_DIR}"/profiling_data/*/; do
        case_name="$(basename "${case_dir}")"
        current_csv="${case_dir}/profile.csv"
        previous_csv="${latest_previous}/profiling_data/${case_name}/profile.csv"

        if [ -f "${current_csv}" ] && [ -f "${previous_csv}" ]; then
            log_info "  Comparing ${case_name}: current vs previous"
            report_html="${RUN_DIR}/reports/${case_name}/comparison.html"
            python3 "${ANALYSIS_SCRIPTS_DIR}/compare_runs.py" \
                "${previous_csv}" \
                "${current_csv}" \
                --output "${report_html}" \
                2>&1 | tee "${RUN_DIR}/reports/${case_name}/compare_runs.log" || {
                    log_error "    compare_runs.py failed for ${case_name}"
                }
        else
            log_info "  No matching profile CSVs for ${case_name}, skipping comparison."
        fi
    done
else
    log_info "  No previous runs found. Skipping comparison."
fi

log_info "Step 5: Comparison reports complete."

# =============================================================================
# Summary
# =============================================================================
log_info "=============================================="
log_info "Benchmark run complete!"
log_info "Results directory: ${RUN_DIR}"
log_info "  Profiling data:  ${RUN_DIR}/profiling_data/"
log_info "  Reports:         ${RUN_DIR}/reports/"
log_info "=============================================="
