#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/run_matlab_batch.sh" "resumeAceDriverPhase4ExtensionAfterRestart();"
