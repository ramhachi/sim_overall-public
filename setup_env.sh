#!/usr/bin/env bash
# FSAE Sim 開発環境セットアップスクリプト (macOS/Linux/WSL)
# 使用法: ./setup_env.sh

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"

echo "========================================"
echo " FSAE Vehicle Dynamics Simulator Setup"
echo "========================================"
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info() { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

detect_triplet() {
    local os arch
    os="$(uname -s)"
    arch="$(uname -m)"

    case "$os" in
        Darwin)
            case "$arch" in
                arm64) echo "arm64-osx" ;;
                x86_64) echo "x64-osx" ;;
                *) error "Unsupported macOS architecture: $arch" ;;
            esac
            ;;
        Linux)
            case "$arch" in
                aarch64|arm64) echo "arm64-linux" ;;
                x86_64) echo "x64-linux" ;;
                *) error "Unsupported Linux architecture: $arch" ;;
            esac
            ;;
        *)
            error "Unsupported host OS: $os"
            ;;
    esac
}

cpu_count() {
    if command -v nproc >/dev/null 2>&1; then
        nproc
    elif command -v sysctl >/dev/null 2>&1; then
        sysctl -n hw.ncpu
    else
        echo 4
    fi
}

persist_vcpkg_root() {
    local shell_rc

    if [[ "$(uname -s)" == "Darwin" ]]; then
        shell_rc="$HOME/.zshrc"
    else
        shell_rc="$HOME/.bashrc"
    fi

    if [[ ! -f "$shell_rc" ]] || ! grep -q 'export VCPKG_ROOT=' "$shell_rc"; then
        echo 'export VCPKG_ROOT="$HOME/vcpkg"' >> "$shell_rc"
    fi
}

configure_cmake_project() {
    local source_dir="$1"
    local build_dir="$2"
    local triplet="$3"
    local manifest_dir="$4"

    cmake -S "$source_dir" -B "$build_dir" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" \
        -DVCPKG_TARGET_TRIPLET="$triplet" \
        -DVCPKG_MANIFEST_DIR="$manifest_dir"
}

setup_vcpkg() {
    info "Setting up vcpkg..."

    if [[ -z "${VCPKG_ROOT:-}" ]]; then
        export VCPKG_ROOT="$HOME/vcpkg"
        persist_vcpkg_root
    fi

    if [[ ! -d "$VCPKG_ROOT" ]]; then
        info "Cloning vcpkg..."
        git clone https://github.com/microsoft/vcpkg.git "$VCPKG_ROOT"
        "$VCPKG_ROOT/bootstrap-vcpkg.sh" -disableMetrics
    else
        info "vcpkg already exists at $VCPKG_ROOT"
    fi

    info "vcpkg setup complete"
}

install_dependencies() {
    local triplet
    triplet="$(detect_triplet)"

    info "Installing vcpkg dependencies for $triplet..."
    cd "$PROJECT_ROOT"
    "$VCPKG_ROOT/vcpkg" install --triplet="$triplet" --x-manifest-root="$PROJECT_ROOT"
    info "Dependencies installed"
}

setup_python() {
    info "Setting up Python environment..."

    cd "$PROJECT_ROOT/python"

    if ! python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
        error "Python 3.10 or newer is required (found $(python3 --version 2>&1))."
        return 1
    fi

    if [[ ! -d ".venv" ]]; then
        python3 -m venv .venv
    fi

    source .venv/bin/activate
    pip install --upgrade pip
    pip install -e ".[dev]"

    info "Python environment ready"
    info "Activate with: source python/.venv/bin/activate"
}

build_cpp() {
    local triplet jobs
    triplet="$(detect_triplet)"
    jobs="$(cpu_count)"

    info "Building core C++ project..."
    configure_cmake_project "$PROJECT_ROOT" "$PROJECT_ROOT/build/release" "$triplet" "$PROJECT_ROOT"
    cmake --build "$PROJECT_ROOT/build/release" -j"$jobs"

    info "Building frame optimization FEM binaries..."
    configure_cmake_project "$PROJECT_ROOT/frame_optimize/cpp" "$PROJECT_ROOT/frame_optimize/cpp/build" "$triplet" "$PROJECT_ROOT"
    cmake --build "$PROJECT_ROOT/frame_optimize/cpp/build" -j"$jobs"
    cmake --install "$PROJECT_ROOT/frame_optimize/cpp/build"

    info "C++ build complete"
    info "Installed FEM binaries to: $PROJECT_ROOT/frame_optimize/bin"
}

run_tests() {
    info "Running tests..."

    cd "$PROJECT_ROOT/build/release"
    ctest --output-on-failure

    info "Tests complete"
}

main() {
    case "${1:-all}" in
        vcpkg)
            setup_vcpkg
            ;;
        deps)
            install_dependencies
            ;;
        python)
            setup_python
            ;;
        build)
            build_cpp
            ;;
        test)
            run_tests
            ;;
        all)
            setup_vcpkg
            install_dependencies
            setup_python
            build_cpp
            run_tests
            ;;
        *)
            echo "Usage: $0 {vcpkg|deps|python|build|test|all}"
            exit 1
            ;;
    esac
}

main "$@"
