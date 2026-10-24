# tests/CMakeLists.txt
# テスト定義

add_executable(fsae_tests
    test_main.cpp
    test_vec3.cpp
    test_rotation.cpp
    test_tmeasy.cpp
    test_bouc_wen.cpp
    test_spline.cpp
    test_magic_formula.cpp
    test_phase5_features.cpp
    test_bicycle_model.cpp
    test_osqp_interface.cpp
    test_mpc.cpp
    test_pou.cpp
    test_drs_system.cpp
    test_frame_flexibility.cpp
    test_steering_torque.cpp
    test_rl_control_safety.cpp
    test_frame_torsion_metric.cpp
    test_vehicle_params.cpp
    test_line_planner.cpp
    test_velocity_profile_actuator.cpp
    test_velocity_profile_io.cpp
    test_driver_benchmark.cpp
    test_fast_lap_vehicle.cpp
    test_physics_regression.cpp
)

target_include_directories(fsae_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/frame_optimize/cpp
)

target_link_libraries(fsae_tests PRIVATE
    fsae_core
    GTest::gtest
    GTest::gtest_main
)
# Unit tests do not require a generated PINN artifact; production keeps the PINN-only guard.
target_compile_definitions(fsae_tests PRIVATE FSAE_TEST_TIRE_MODEL_TYPE="tmeasy")

# CTestに登録
include(GoogleTest)
gtest_discover_tests(fsae_tests)
