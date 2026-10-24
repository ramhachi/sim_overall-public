#!/bin/bash
# Docker entrypoint script

set -e

build_cpp() {
    mkdir -p /workspace/build
    cd /workspace/build
    cmake .. -G Ninja -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
    cmake --build . --parallel
}

# CMake設定 & ビルド
if [ "$1" = "build" ]; then
    echo "=== Building FSAE Simulator ==="
    build_cpp
    echo "=== Build Complete ==="

elif [ "$1" = "test" ]; then
    echo "=== Running Tests ==="
    build_cpp
    cd /workspace/build
    ctest --output-on-failure
    cd /workspace
    pytest tests/ -v
    echo "=== Tests Complete ==="

elif [ "$1" = "test-cpp" ]; then
    echo "=== Running C++ Tests ==="
    build_cpp
    cd /workspace/build
    ctest --output-on-failure
    echo "=== C++ Tests Complete ==="

elif [ "$1" = "test-python" ]; then
    echo "=== Running Python Tests ==="
    build_cpp
    cd /workspace
    pytest tests/ -v
    echo "=== Tests Complete ==="

elif [ "$1" = "dev" ]; then
    echo "=== Development Mode ==="
    exec /bin/bash

else
    exec "$@"
fi
