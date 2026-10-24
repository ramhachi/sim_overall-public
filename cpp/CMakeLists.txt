# Docker環境でのビルドに対応
cmake_minimum_required(VERSION 3.10)
project(fsae_simulator)

# 最適化オプション（ユーザー要望対応）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT DEFINED FSAE_EFFECTIVE_CPU_ARCH_FLAGS)
    set(FSAE_EFFECTIVE_CPU_ARCH_FLAGS "")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # 実行速度を最大限にするが、NaN/Infチェックは残す (-O3)
    #   -O3               : 高度な最適化（-ffast-mathは含まれないため安全）
    #   host CPU flags    : 親CMakeで解決した native 最適化フラグ
    #   -funroll-loops    : ループ展開（RK4積分ループに有効）
    #   -fomit-frame-pointer : フレームポインタ省略でレジスタ1本節約
    #   -fno-semantic-interposition : インプロセス呼び出しを直接参照に変換
    #   -fdata-sections -ffunction-sections : 不要シンボルをリンカが除去できるよう分割
    #   -flto=auto        : 全コア使用のLink Time Optimization
    #   -DNDEBUG          : assert() を無効化
    set(FSAE_RELEASE_CXX_FLAGS
        "-O3 -funroll-loops -fomit-frame-pointer -fno-semantic-interposition -fdata-sections -ffunction-sections -flto=auto -DNDEBUG")
    if(NOT FSAE_EFFECTIVE_CPU_ARCH_FLAGS STREQUAL "")
        string(APPEND FSAE_RELEASE_CXX_FLAGS " ${FSAE_EFFECTIVE_CPU_ARCH_FLAGS}")
    endif()
    set(CMAKE_CXX_FLAGS_RELEASE "${FSAE_RELEASE_CXX_FLAGS}")
    # リンク時: LTO + 未使用セクション除去
    if(APPLE)
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE    "-flto=auto -Wl,-dead_strip")
        set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "-flto=auto -Wl,-dead_strip")
    else()
        set(CMAKE_EXE_LINKER_FLAGS_RELEASE    "-flto=auto -Wl,--gc-sections")
        set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "-flto=auto -Wl,--gc-sections")
    endif()
    if(FSAE_EFFECTIVE_CPU_ARCH_FLAGS STREQUAL "")
        message(STATUS "High-performance build: -O3 -funroll-loops -flto=auto (portable CPU flags)")
    else()
        message(STATUS "High-performance build: -O3 ${FSAE_EFFECTIVE_CPU_ARCH_FLAGS} -funroll-loops -flto=auto")
    endif()
endif()

# 依存パッケージを探す
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(EnTT QUIET)  # EnTTは任意
find_package(GTest QUIET)  # テストは任意
find_package(osqp QUIET)   # Phase 8a: MPC用QP solver

add_library(fsae_core STATIC)

target_sources(fsae_core PRIVATE
    src/math/rotation.cpp
    src/math/spline.cpp
    src/physics/aero/aero_map_4d.cpp
)

target_include_directories(fsae_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(fsae_core PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    spdlog::spdlog
)

# Releaseビルド専用の定義
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(fsae_core PUBLIC
        EIGEN_NO_DEBUG                          # Eigen境界チェック無効
        EIGEN_NO_STATIC_ASSERT                  # Eigen static_assert無効
        SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_WARN   # spdlogをWARN以上のみに絞る
    )
endif()

# EnTTがある場合のみリンク
if(TARGET EnTT::EnTT)
    target_link_libraries(fsae_core PUBLIC EnTT::EnTT)
endif()

# autodiffがある場合のみリンク
if(TARGET autodiff::autodiff)
    target_link_libraries(fsae_core PUBLIC autodiff::autodiff)
    target_compile_definitions(fsae_core PUBLIC HAS_AUTODIFF=1)
endif()

# OSQPがある場合のみリンク（Phase 8a: MPC, OSQP v1.0）
find_package(osqp QUIET)
if(TARGET osqp::osqpstatic)
    target_link_libraries(fsae_core PUBLIC osqp::osqpstatic)
    target_compile_definitions(fsae_core PUBLIC HAS_OSQP=1)
    message(STATUS "OSQP found (static) and linked for MPC support")
elseif(TARGET osqp::osqp)
    target_link_libraries(fsae_core PUBLIC osqp::osqp)
    target_compile_definitions(fsae_core PUBLIC HAS_OSQP=1)
    message(STATUS "OSQP found (shared) and linked for MPC support")
endif()

# 実行ファイル
add_executable(fsae_sim src/main.cpp)
target_link_libraries(fsae_sim PRIVATE fsae_core)

# 14DOFフルビークルモデル
add_executable(fsae_14dof src/main_14dof.cpp)
target_link_libraries(fsae_14dof PRIVATE fsae_core)

# 3DOFプレーナーモデル（デバッグ用）
add_executable(fsae_3dof src/main_3dof.cpp)
target_link_libraries(fsae_3dof PRIVATE fsae_core)

# 6DOFモデル（3DOF + ロール/ピッチ/ヒーブ）
add_executable(fsae_6dof src/main_6dof.cpp)
target_link_libraries(fsae_6dof PRIVATE fsae_core)

# 6DOF平衡テスト
add_executable(test_6dof_eq src/test_6dof_equilibrium.cpp)
target_link_libraries(test_6dof_eq PRIVATE fsae_core)

# 6DOF速度テスト
add_executable(test_6dof_vel src/test_6dof_velocity.cpp)
target_link_libraries(test_6dof_vel PRIVATE fsae_core)

# 8DOFモデルテスト
add_executable(test_8dof src/test_8dof.cpp)
target_link_libraries(test_8dof PRIVATE fsae_core)

# 8DOFデモ
add_executable(fsae_8dof src/main_8dof.cpp)
target_link_libraries(fsae_8dof PRIVATE fsae_core)

# ARBテスト
add_executable(test_arb src/test_arb.cpp)
target_link_libraries(test_arb PRIVATE fsae_core)

# ARBデバッグテスト
add_executable(test_arb_debug src/test_arb_debug.cpp)
target_link_libraries(test_arb_debug PRIVATE fsae_core)

# Bouc-Wenダンパー統合テスト
add_executable(test_bouc_wen_vehicle src/test_bouc_wen_vehicle.cpp)
target_link_libraries(test_bouc_wen_vehicle PRIVATE fsae_core)

# LuGre摩擦統合テスト（8DOF_Full）
add_executable(test_lugre_vehicle src/test_lugre_vehicle.cpp)
target_link_libraries(test_lugre_vehicle PRIVATE fsae_core)

# 14DOF新モデルテスト
add_executable(test_14dof_new src/test_14dof_new.cpp)
target_link_libraries(test_14dof_new PRIVATE fsae_core)

# 14DOF Rosenbrockテスト
add_executable(test_14dof_rosenbrock src/test_14dof_rosenbrock.cpp)
target_link_libraries(test_14dof_rosenbrock PRIVATE fsae_core)

# オープンループドライバーテスト（Phase 6b）
add_executable(test_open_loop_driver src/test_open_loop_driver.cpp)
target_link_libraries(test_open_loop_driver PRIVATE fsae_core)

# トラックモデルテスト（Phase 6c）
add_executable(test_track_model src/test_track_model.cpp)
target_link_libraries(test_track_model PRIVATE fsae_core)

# 速度プロファイル生成テスト（Phase 6d）
add_executable(test_velocity_profile src/test_velocity_profile.cpp)
target_link_libraries(test_velocity_profile PRIVATE fsae_core)

# PID経路追従ドライバーテスト（Phase 7a）
add_executable(test_pid_driver src/test_pid_driver.cpp)
target_link_libraries(test_pid_driver PRIVATE fsae_core)

# ラップタイムシミュレーション（Phase 7b）
add_executable(laptime_sim src/main_laptime_sim.cpp)
target_link_libraries(laptime_sim PRIVATE fsae_core)

if(BUILD_TESTS)
    add_test(
        NAME laptime_fast_eval_rejects_torque_mode
        COMMAND ${CMAKE_COMMAND}
            -DLAPTIME_SIM=$<TARGET_FILE:laptime_sim>
            -DSOURCE_ROOT=${CMAKE_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_fast_eval_rejects_torque_mode.cmake
    )
endif()

# Driver capability benchmark
add_executable(driver_benchmark src/main_driver_benchmark.cpp)
target_link_libraries(driver_benchmark PRIVATE fsae_core)

# Target-speed-only ideal laptime bound
add_executable(ideal_speed_target src/main_ideal_speed_target.cpp)
target_link_libraries(ideal_speed_target PRIVATE fsae_core)

# PID vs MPC 比較シミュレーション（Phase 8b）
add_executable(mpc_comparison src/main_mpc_comparison.cpp)
target_link_libraries(mpc_comparison PRIVATE fsae_core)

# MPCパラメータチューニング（Phase 8c）
add_executable(mpc_tuning src/main_mpc_tuning.cpp)
target_link_libraries(mpc_tuning PRIVATE fsae_core)

# 4Dエアロマップテスト
add_executable(test_aero_map_4d src/test_aero_map_4d.cpp)
target_link_libraries(test_aero_map_4d PRIVATE fsae_core)

# 4Dエアロマップ統合テスト
add_executable(test_aero_integration src/test_aero_integration.cpp)
target_link_libraries(test_aero_integration PRIVATE fsae_core)

# エンジントルクカーブテスト
add_executable(test_engine_torque src/test_engine_torque.cpp)
target_link_libraries(test_engine_torque PRIVATE fsae_core)

# LSD（Limited Slip Differential）テスト
add_executable(test_lsd src/test_lsd.cpp)
target_link_libraries(test_lsd PRIVATE fsae_core)

# LSD統合テスト（14DOF車両）
add_executable(test_lsd_14dof src/test_lsd_14dof.cpp)
target_link_libraries(test_lsd_14dof PRIVATE fsae_core)

# NNタイヤモデル統合テスト
add_executable(test_nn_tire tests/test_nn_tire_integration.cpp)
target_link_libraries(test_nn_tire PRIVATE fsae_core GTest::gtest GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(test_nn_tire)

# Phase A1+A2 Integration Test
add_executable(test_integration tests/test_phase_a1_a2_integration.cpp)
target_link_libraries(test_integration PRIVATE fsae_core GTest::gtest GTest::gtest_main)
# Keep the vehicle integration test independent of generated PINN artifacts.
target_compile_definitions(test_integration PRIVATE FSAE_TEST_TIRE_MODEL_TYPE="tmeasy")
gtest_discover_tests(test_integration)

# PID Driver Torque Mode Test
add_executable(test_pid_torque tests/test_pid_driver_torque_mode.cpp)
target_link_libraries(test_pid_torque PRIVATE fsae_core GTest::gtest GTest::gtest_main)
gtest_discover_tests(test_pid_torque)

# Preview LPV-LQR driver tests
add_executable(test_lpv_lqr_driver tests/test_lpv_lqr_driver.cpp)
target_link_libraries(test_lpv_lqr_driver PRIVATE fsae_core GTest::gtest GTest::gtest_main)
gtest_discover_tests(test_lpv_lqr_driver)

# RL学習用TCPサーバー
add_executable(fsae_rl_server src/fsae_rl_server.cpp)
target_link_libraries(fsae_rl_server PRIVATE fsae_core)

# RL Actor一致性チェック
add_executable(rl_actor_check src/main_rl_actor_check.cpp)
target_link_libraries(rl_actor_check PRIVATE fsae_core)

# C++ Profile
add_executable(prof_fsae src/prof_fsae.cpp)
target_link_libraries(prof_fsae PRIVATE fsae_core)

add_executable(prof_nn_tire src/prof_nn_tire.cpp)
target_link_libraries(prof_nn_tire PRIVATE fsae_core)
