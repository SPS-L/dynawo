#!/bin/bash
# Script Author: Sustainable Power Systems Lab (SPS-L) - https://sps-lab.org
# Contact: info@sps-lab.org

set -euo pipefail

DYNAWO_REPO_URL="https://github.com/SPS-L/dynawo.git"
DYNAWO_BRANCH="1_rt-monitor"
DYNAWO_HOME="/tmp/dynawo"

clone_or_update_dynawo_repo() {
  if [ -d "${DYNAWO_HOME}/.git" ]; then
    echo "Updating existing Dynawo clone at ${DYNAWO_HOME}"
    pushd "${DYNAWO_HOME}" >/dev/null
    git remote set-url origin "${DYNAWO_REPO_URL}"
    git fetch --all --prune
    # Ensure we are on the desired branch and hard reset to remote tracking
    git checkout "${DYNAWO_BRANCH}" 2>/dev/null || git checkout -b "${DYNAWO_BRANCH}" "origin/${DYNAWO_BRANCH}" || true
    git reset --hard "origin/${DYNAWO_BRANCH}" || true
    popd >/dev/null
  else
    echo "Cloning Dynawo repository into ${DYNAWO_HOME}"
    rm -rf "${DYNAWO_HOME}"
    git clone --depth=1 --branch "${DYNAWO_BRANCH}" "${DYNAWO_REPO_URL}" "${DYNAWO_HOME}"
  fi
}

create_dynawo_env() {
  local dynawo_home="$1"
  cat <<EOF > "${dynawo_home}/myEnvDynawo.sh"
#!/bin/bash
export DYNAWO_HOME="${dynawo_home}"

export DYNAWO_SRC_OPENMODELICA=\$DYNAWO_HOME/OpenModelica/Source
export DYNAWO_INSTALL_OPENMODELICA=\$DYNAWO_HOME/OpenModelica/Install
export DYNAWO_LOCALE=en_GB
export DYNAWO_RESULTS_SHOW=true
export DYNAWO_BROWSER=firefox
export DYNAWO_PYTHON_COMMAND=python3
export DYNAWO_NB_PROCESSORS_USED=4
export DYNAWO_BUILD_TYPE=RelWithDebInfo
\$DYNAWO_HOME/util/envDynawo.sh \$@
EOF
  chmod +x "${dynawo_home}/myEnvDynawo.sh"
}


build_dynawo_all() {
  local dynawo_home="$1"
  echo "Building Dynawo with build-user..."
  pushd "${dynawo_home}" >/dev/null
  ./myEnvDynawo.sh build-user
  ./myEnvDynawo.sh deploy
  popd >/dev/null
}

build_dynawo_core() {
  local dynawo_home="$1"
  echo "Building Dynawo with build-dynawo-core..."
  pushd "${dynawo_home}" >/dev/null
  ./myEnvDynawo.sh build-dynawo-core
  ./myEnvDynawo.sh deploy
  popd >/dev/null
}

show_help() {
  cat <<EOF
Dynawo Build Script
Author: Sustainable Power Systems Lab (SPS-L) - https://sps-lab.org
Contact: info@sps-lab.org

USAGE:
  $0 [COMMAND]

COMMANDS:
  help          Show this help message
  all           Build Dynawo with build-user and then deploy (complete build)
  core          Build Dynawo with build-dynawo-core and then deploy (core only)
  clean         Clean the Dynawo build directory

DESCRIPTION:
  This script builds Dynawo from the SPS-L/dynawo repository (branch: ${DYNAWO_BRANCH}).
  
  - all:  Complete Dynawo build including all components and user models
  - core: Core Dynawo build focusing on essential components only
  - clean: Remove the Dynawo installation directory for a fresh build
  
  The script automatically:
  - Clones or updates the repository at ${DYNAWO_HOME}
  - Creates the necessary environment configuration
  - Builds and deploys the selected components

EXAMPLES:
  $0 help           # Show this help
  $0 all            # Build complete Dynawo
  $0 core           # Build core Dynawo only
  $0 clean          # Clean the build directory

NOTES:
  - All builds are performed in ${DYNAWO_HOME}
  - The script will clone/update the repository as needed
  - Build artifacts are deployed automatically
  - Repository: ${DYNAWO_REPO_URL}
  - Branch: ${DYNAWO_BRANCH}

EOF
}

setup_environment() {
  clone_or_update_dynawo_repo
  create_dynawo_env "${DYNAWO_HOME}"
}

clean_build() {
  if [ -d "${DYNAWO_HOME}" ]; then
    echo "Removing Dynawo directory at ${DYNAWO_HOME}..."
    rm -rf "${DYNAWO_HOME}"
    echo "Clean completed. Run 'all' or 'core' to rebuild."
  else
    echo "Dynawo directory not found at ${DYNAWO_HOME}. Nothing to clean."
  fi
}

main() {
  local command="${1:-help}"
  
  case "${command}" in
    help|--help|-h)
      show_help
      exit 0
      ;;
    all)
      echo "Building Dynawo (complete build)..."
      setup_environment
      build_dynawo_all "${DYNAWO_HOME}"
      echo "✓ Dynawo build completed successfully!"
      echo ""
      echo "To run Dynawo simulations, use:"
      echo "  cd ${DYNAWO_HOME}"
      echo "  ./myEnvDynawo.sh jobs-with-curves examples/DynaWaltz/IEEE14/IEEE14_GeneratorDisconnections/IEEE14.jobs"
      ;;
    core)
      echo "Building Dynawo (core only)..."
      setup_environment
      build_dynawo_core "${DYNAWO_HOME}"
      echo "✓ Dynawo core build completed successfully!"
      echo ""
      echo "To run Dynawo simulations, use:"
      echo "  cd ${DYNAWO_HOME}"
      echo "  ./myEnvDynawo.sh jobs-with-curves examples/DynaWaltz/IEEE14/IEEE14_GeneratorDisconnections/IEEE14.jobs"
      ;;
    clean)
      clean_build
      ;;
    *)
      echo "Error: Unknown command '${command}'"
      echo "Use '$0 help' to see available commands."
      exit 1
      ;;
  esac
}

main "$@"

