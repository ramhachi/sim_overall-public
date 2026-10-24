#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ]; then
    echo "usage: easystart_container_build.sh <build-signature> <generator> <cpu-profile> [targets...]" >&2
    exit 64
fi

build_signature="$1"
generator="$2"
cpu_profile="$3"
shift 3

build_dir="/workspace/build"
profile_file="${build_dir}/.easystart_build_profile"

mkdir -p "${build_dir}"

if [ ! -f "${profile_file}" ] || [ "$(cat "${profile_file}")" != "${build_signature}" ]; then
    find "${build_dir}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
fi

if ! cmake -S /workspace -B "${build_dir}" -G "${generator}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DFSAE_CPU_OPTIMIZATION_PROFILE="${cpu_profile}"; then
    exit 2
fi

echo -n "${build_signature}" > "${profile_file}"

if [ "$#" -eq 0 ]; then
    cmake --build "${build_dir}" --parallel "$(nproc)"
else
    cmake --build "${build_dir}" --parallel "$(nproc)" --target "$@"
fi
