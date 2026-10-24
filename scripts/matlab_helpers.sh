#!/usr/bin/env bash

resolve_matlab_bin() {
  if [[ -n "${MATLAB_BIN:-}" ]]; then
    if [[ -x "${MATLAB_BIN}" ]]; then
      printf '%s\n' "${MATLAB_BIN}"
      return 0
    fi
    echo "[matlab] ERROR: MATLAB_BIN is set but not executable: ${MATLAB_BIN}" >&2
    return 1
  fi

  local candidate
  if candidate="$(resolve_matlab_bin_from_fsae_alias 2>/dev/null)"; then
    printf '%s\n' "${candidate}"
    return 0
  fi

  if command -v matlab >/dev/null 2>&1; then
    command -v matlab
    return 0
  fi

  local -a candidates=()
  shopt -s nullglob
  candidates+=("/Applications"/MATLAB*.app/bin/matlab)
  candidates+=("${HOME}"/Applications/MATLAB*.app/bin/matlab)
  candidates+=("/usr/local"/MATLAB/*/bin/matlab)
  shopt -u nullglob

  for (( idx=${#candidates[@]}-1; idx>=0; idx-- )); do
    candidate="${candidates[idx]}"
    if [[ -x "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

resolve_matlab_bin_from_fsae_alias() {
  local config line candidate
  local -a configs=(
    "${HOME}/.zshrc"
    "${HOME}/.zprofile"
    "${HOME}/.bashrc"
    "${HOME}/.bash_profile"
    "${HOME}/.profile"
  )

  for config in "${configs[@]}"; do
    [[ -r "${config}" ]] || continue
    while IFS= read -r line; do
      [[ "${line}" == *matlab* ]] || continue
      [[ "${line}" == *fsae* || "${line}" == *FSAE* || "${line}" == *MATLAB* ]] || continue

      if [[ "${line}" =~ [\"\']?([^\"\'\ ]*MATLAB[^\"\'\ ]*\.app/bin/matlab)[\"\']? ]]; then
        candidate="${BASH_REMATCH[1]}"
      elif [[ "${line}" =~ [\"\']?(/[^\"\'\ ]*/matlab)[\"\']? ]]; then
        candidate="${BASH_REMATCH[1]}"
      else
        continue
      fi

      if [[ -x "${candidate}" ]]; then
        printf '%s\n' "${candidate}"
        return 0
      fi
    done < "${config}"
  done

  return 1
}

print_matlab_resolution_help() {
  local context="${1:-script}"
  cat >&2 <<EOF
[${context}] ERROR: Could not find a MATLAB executable.

Tried, in order:
  1. \$MATLAB_BIN
  2. MATLAB path parsed from an fsae/FSAE shell alias or function
  3. matlab on PATH
  4. Common macOS install locations such as:
     /Applications/MATLAB_*.app/bin/matlab

Fix one of these, then rerun.
You can also override detection explicitly, for example:
  MATLAB_BIN="/Applications/MATLAB_R2024b.app/bin/matlab" bash scripts/run_matlab_batch.sh
EOF
}
