#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/matlab_helpers.sh"

if ! MATLAB_BIN="$(resolve_matlab_bin)"; then
  print_matlab_resolution_help "run_matlab_batch"
  exit 1
fi

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 \"MATLAB_COMMANDS\"" >&2
  exit 1
fi

MATLAB_COMMANDS="$1"
"${MATLAB_BIN}" -batch "cd('${PROJECT_ROOT}'); addpath(fullfile('${PROJECT_ROOT}','matlab')); addpath(fullfile('${PROJECT_ROOT}','matlab','rl_driver')); ${MATLAB_COMMANDS}"
