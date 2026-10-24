# Docker環境でのビルドに対応したCMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(fsae_sim VERSION 0.1.0 LANGUAGES CXX)

# C++20必須
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CPU最適化プロファイル
option(FSAE_ENABLE_HOST_CPU_OPTIMIZATION "Enable host CPU specific release optimizations (not portable across machines)" OFF)
set(FSAE_CPU_OPTIMIZATION_PROFILE "auto" CACHE STRING
    "Release CPU optimization profile (auto, x86-native, arm-native, generic-native, portable)")
set_property(CACHE FSAE_CPU_OPTIMIZATION_PROFILE PROPERTY STRINGS
    auto x86-native arm-native generic-native portable)

set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "")
set(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE "portable")

if(FSAE_ENABLE_HOST_CPU_OPTIMIZATION AND NOT MSVC)
    set(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE "${FSAE_CPU_OPTIMIZATION_PROFILE}")
    if(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE STREQUAL "auto")
        string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" FSAE_SYSTEM_PROCESSOR_LOWER)
        if(FSAE_SYSTEM_PROCESSOR_LOWER MATCHES "^(arm64|aarch64)")
            set(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE "arm-native")
        elseif(FSAE_SYSTEM_PROCESSOR_LOWER MATCHES "^(x86_64|amd64|i[3-6]86)")
            set(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE "x86-native")
        else()
            set(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE "generic-native")
        endif()
    endif()

    if(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE STREQUAL "x86-native")
        set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "-march=native -mtune=native")
    elseif(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE STREQUAL "arm-native")
        set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "-mcpu=native")
    elseif(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE STREQUAL "generic-native")
        set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "-march=native")
    elseif(FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE STREQUAL "portable")
        set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "")
    else()
        message(FATAL_ERROR "Unknown FSAE_CPU_OPTIMIZATION_PROFILE: ${FSAE_CPU_OPTIMIZATION_PROFILE}")
    endif()
endif()

# コンパイラ警告設定
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存ライブラリ（find_package）
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)

# EnTT（ヘッダオンリー）
find_package(EnTT CONFIG QUIET)
if(NOT EnTT_FOUND)
    # フォールバック: インクルードパスから探す
    find_path(ENTT_INCLUDE_DIR entt/entt.hpp
        PATHS /opt/entt/single_include /usr/local/include /usr/include
    )
    if(ENTT_INCLUDE_DIR)
        add_library(EnTT::EnTT INTERFACE IMPORTED)
        target_include_directories(EnTT::EnTT INTERFACE ${ENTT_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "EnTT not found")
    endif()
endif()

# autodiff（ヘッダオンリー）
find_package(autodiff CONFIG QUIET)
if(NOT autodiff_FOUND)
    find_path(AUTODIFF_INCLUDE_DIR autodiff/forward/dual.hpp
        PATHS /opt/autodiff /usr/local/include /usr/include
    )
    if(AUTODIFF_INCLUDE_DIR)
        add_library(autodiff::autodiff INTERFACE IMPORTED)
        target_include_directories(autodiff::autodiff INTERFACE ${AUTODIFF_INCLUDE_DIR})
    else()
        message(WARNING "autodiff not found - some features will be disabled")
        set(AUTODIFF_FOUND FALSE)
    endif()
else()
    set(AUTODIFF_FOUND TRUE)
endif()

# オプション
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

# cpp/CMakeLists.txt 内の gtest_discover_tests() より先に testing を有効化する
if(BUILD_TESTS)
    enable_testing()
endif()

# サブディレクトリ
add_subdirectory(cpp)

# テスト
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    add_subdirectory(cpp/tests)
endif()

# Pythonバインディング
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 CONFIG QUIET)
    if(pybind11_FOUND)
        add_subdirectory(cpp/bindings)
    else()
        message(WARNING "pybind11 not found - Python bindings disabled")
    endif()
endif()

# 設定サマリー
message(STATUS "")
message(STATUS "=== FSAE Simulator Configuration ===")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Host CPU optimization: ${FSAE_ENABLE_HOST_CPU_OPTIMIZATION}")
message(STATUS "  CPU profile: ${FSAE_RESOLVED_CPU_OPTIMIZATION_PROFILE}")
message(STATUS "  CPU flags: ${FSAE_EFFECTIVE_CPU_ARCH_FLAGS}")
message(STATUS "  Eigen3: ${EIGEN3_VERSION_STRING}")
message(STATUS "  autodiff: ${AUTODIFF_FOUND}")
message(STATUS "  Tests: ${BUILD_TESTS}")
message(STATUS "  Python bindings: ${BUILD_PYTHON_BINDINGS}")
message(STATUS "")
