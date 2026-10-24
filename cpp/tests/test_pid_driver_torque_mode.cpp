/// @file test_pid_driver_torque_mode.cpp
/// @brief PID経路追従ドライバーのトルクモード用テスト（Phase A2）

#include <gtest/gtest.h>
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include "fsae/vehicle/vehicle_params.hpp"
#include "fsae/driver/pid_driver.hpp"
#include "fsae/track/track_model.hpp"
#include "fsae/laptime/velocity_profile.hpp"

using namespace fsae::vehicle;
using namespace fsae::driver;
using namespace fsae::track;
using namespace fsae::laptime;

class PIDDriverTorqueTest : public ::testing::Test {
protected:
    VehicleParams params;
    CurvatureBasedTrack track = create_acceleration_track(100.0);
    std::vector<VelocityPoint> velocity_profile;

    void SetUp() override {
        params = VehicleParams::default_fsae();
        params.steering.torque_mode = true;
        
        velocity_profile.push_back({0.0, 10.0});
        velocity_profile.push_back({100.0, 10.0});
    }
};

/// Test 1: 内側ループによるトルク出力確認
TEST_F(PIDDriverTorqueTest, InnerLoopTorqueOutput) {
    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.Kp_steer_inner = 50.0;
    gains.Kd_steer_inner = 5.0;
    gains.R_steer = 4.5;
    gains.max_steer_torque = 25.0;
    
    PIDPathFollowingDriver driver(track, velocity_profile, gains);
    
    State14DOF state;
    // 車両は左に1mずれている状態（y = 1.0）
    state.x = 0.0;
    state.y = 1.0; 
    state.psi = 0.0;
    state.vx = 10.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;
    
    // 現在の横偏差 e_lat = 1.0 なので、目標舵角 delta_target は正（右旋回）となる
    // したがって、T_driver は正（右旋回トルク）が出力されるはず
    DriverInput input = driver.compute_control(state, 0.001); // 初期化
    input = driver.compute_control(state, 0.010);             // 微分計算のため時間進行
    
    EXPECT_GT(input.steering, 0.0) << "Should output positive torque to steer right";
    EXPECT_LE(input.steering, gains.max_steer_torque) << "Torque should not exceed limit";
}

/// Test 2: ステアリング角速度によるダンピング効果
TEST_F(PIDDriverTorqueTest, InnerLoopDampingEffect) {
    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.Kp_steer_inner = 50.0;
    gains.Kd_steer_inner = 5.0; // 微分ゲイン(ダンピング)
    gains.max_steer_torque = 100.0; // クランプされないように上限を上げる
    
    PIDPathFollowingDriver driver(track, velocity_profile, gains);
    
    State14DOF static_state;
    static_state.x = 0; static_state.y = 1.0; static_state.psi = 0;
    static_state.vx = 10.0; static_state.delta_sw = 0.0; static_state.ddelta_sw = 0.0;
    
    State14DOF dynamic_state = static_state;
    dynamic_state.ddelta_sw = 2.0; // 既に右へ速く切っている状態
    
    driver.reset();
    DriverInput input_static = driver.compute_control(static_state, 0.001);
    
    driver.reset();
    DriverInput input_dynamic = driver.compute_control(dynamic_state, 0.001);
    
    // ダンピング項 (-Kd * ddelta_sw) により、dynamic状態でのトルクは小さくなるはず
    EXPECT_LT(input_dynamic.steering, input_static.steering) 
        << "Torque should be reduced by inner loop damping when ddelta_sw > 0";
}

/// Test 3: 角度モードとの後方互換性
TEST_F(PIDDriverTorqueTest, AngleModeCompatibility) {
    PIDGains gains = conservative_gains();
    gains.torque_mode = false; // 角度モード
    
    PIDPathFollowingDriver driver(track, velocity_profile, gains);
    
    State14DOF state;
    state.x = 0; state.y = 1.0; state.psi = 0; state.vx = 10.0;
    
    DriverInput input = driver.compute_control(state, 0.001);
    
    // 角度モードでは max_steer (例: 0.35 rad) の範囲内で目標舵角がそのまま入る
    EXPECT_LT(std::abs(input.steering), 1.0) << "Should output angle (rad) in angle mode";
}

TEST_F(PIDDriverTorqueTest, LinePlannerFeedsPlannedCurvatureToSteeringFF) {
    CurvatureBasedTrack circle_track = create_circle_track(20.0, 3.0);
    std::vector<VelocityPoint> profile;
    profile.push_back({0.0, 10.0});
    profile.push_back({circle_track.length(), 10.0});

    PIDGains gains = conservative_gains();
    gains.torque_mode = false;
    gains.Kp_steer = 0.0;
    gains.Kd_steer = 0.0;
    gains.Ki_steer = 0.0;
    gains.Kh_steer = 0.0;
    gains.Kff_steer = 1.0;
    gains.max_steer = 1.0;
    gains.line_planner.enabled = true;
    gains.line_planner.safety_margin = 0.25;
    gains.line_planner.offset_fraction = 0.8;
    gains.line_planner.preview_distance = 8.0;
    gains.line_planner.smoothing_distance = 1.0;
    gains.line_planner.track_geometry = true;

    PIDPathFollowingDriver driver(circle_track, profile, gains);

    const double s = 20.0;
    const auto point = circle_track.get_point(s);
    State14DOF state;
    state.x = point.y;
    state.y = point.x;
    state.psi = M_PI / 2.0 - point.theta;
    state.vx = 10.0;

    const auto input = driver.compute_control(state, 0.001);
    const auto geometry = compute_planned_line_geometry(circle_track, s, gains.line_planner);
    const double centerline_ff = -gains.wheelbase * circle_track.get_curvature(s);
    const double planned_ff = -gains.wheelbase * geometry.curvature;

    EXPECT_NEAR(input.steering, planned_ff, 1e-6);
    EXPECT_LT(input.steering, centerline_ff)
        << "Inside planned line on a left circle should demand more left steer than centerline FF.";
    EXPECT_NEAR(driver.get_last_debug().target_curvature, geometry.curvature, 1e-9);
    EXPECT_NEAR(driver.get_last_debug().target_heading_error, 0.0, 0.02);
}

/// Test 4: 操舵アクチュエータの到達性で目標速度が下がる
TEST_F(PIDDriverTorqueTest, SteeringActuatorSpeedCapReducesTargetSpeed) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;
    torque_params.steering.R_steer = 4.5;

    CurvatureBasedTrack tight_track = create_circle_track(8.0, 3.0);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({tight_track.length(), 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.max_tire_steer_rate = 0.10;
    gains.torque_speed_cap_preview_distance = 6.0;

    PIDPathFollowingDriver driver(tight_track, fast_profile, gains, torque_params);

    auto start = tight_track.get_point(0.0);
    State14DOF state;
    state.x = start.y;
    state.y = start.x;
    state.psi = M_PI / 2.0 - start.theta;
    state.vx = 20.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_LT(debug.steering_actuator_speed_cap, 20.0);
    EXPECT_LT(debug.target_speed_final, debug.target_speed_preview);
    EXPECT_NEAR(debug.actual_delta_tire, 0.0, 1e-12);
    EXPECT_GT(std::abs(debug.predicted_delta_tire), 0.0);
    EXPECT_LT(std::abs(debug.predicted_delta_tire), std::abs(debug.delta_target));
    EXPECT_GT(std::abs(debug.required_delta_preview), 0.05);
    EXPECT_NEAR(debug.preview_delta_gap, debug.required_delta_preview, 1e-12);
    EXPECT_GT(debug.time_to_reach_delta, 0.0);
    EXPECT_GE(debug.torque_headroom, 0.0);
    EXPECT_LE(debug.torque_headroom, 1.0);
}

/// Test 5: トラック境界マージン不足で目標速度が下がる
TEST_F(PIDDriverTorqueTest, TrackMarginSpeedCapReducesTargetSpeed) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    CurvatureBasedTrack narrow_track = create_acceleration_track(100.0, 3.0);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 18.0});
    fast_profile.push_back({100.0, 18.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.track_margin_speed_cap = 6.0;
    gains.track_margin_speed_release = 1.0;
    gains.torque_speed_cap_preview_distance = 0.0;

    PIDPathFollowingDriver driver(narrow_track, fast_profile, gains, torque_params);

    State14DOF state;
    state.x = 1.35; // map_y; 0.15 m margin on a 3 m wide straight track
    state.y = 0.0;  // map_x
    state.psi = M_PI / 2.0;
    state.vx = 12.0;

    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_LT(debug.track_margin_speed_cap, 18.0);
    EXPECT_LT(debug.target_speed_final, debug.target_speed_preview);
}

/// Test 6: 外向きヘディングで境界に近づく時は予測マージンで早めに減速する
TEST_F(PIDDriverTorqueTest, ProjectedTrackMarginSpeedCapAnticipatesOutwardHeading) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    CurvatureBasedTrack wide_track = create_acceleration_track(100.0, 5.0);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 18.0});
    fast_profile.push_back({100.0, 18.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.track_margin_speed_cap = 6.0;
    gains.track_margin_speed_release = 1.0;
    gains.track_margin_preview_time = 0.50;

    PIDPathFollowingDriver driver(wide_track, fast_profile, gains, torque_params);

    State14DOF state;
    state.x = 1.20; // map_y, margin is still 1.30 m on a 5 m wide straight.
    state.y = 0.0;
    state.psi = M_PI / 2.0 - 0.20; // positive Frenet heading: moving toward positive n.
    state.vx = 15.0;

    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_GT(debug.track_margin_outward_rate, 0.0);
    EXPECT_LT(debug.track_margin_effective_margin, gains.track_margin_speed_release);
    EXPECT_LT(debug.track_margin_speed_cap, 18.0);
    EXPECT_LT(debug.target_speed_final, debug.target_speed_preview);
}

/// Test 7: Ki override は auto tune の Kp/Kd 正規化を無効化しない
TEST_F(PIDDriverTorqueTest, KiOverridePreservesAutoTunedKpKd) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.C_steer_torque = 0.3;

    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.inner_loop_auto_tune = true;
    gains.inner_loop_omega_n = 30.0;
    gains.inner_loop_zeta = 0.70;
    gains.inner_loop_ki_override = 7.0;
    gains.Kp_steer_inner = 999.0;
    gains.Kd_steer_inner = 999.0;
    gains.max_steer_torque = 100.0;

    PIDPathFollowingDriver driver(track, velocity_profile, gains, torque_params);

    State14DOF state;
    state.x = 0.0;
    state.y = 1.0;
    state.psi = 0.0;
    state.vx = 10.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    ASSERT_GT(std::abs(debug.sw_error), 1e-9);
    const double effective_stiffness = debug.T_driver_raw / debug.sw_error;
    const double expected_kp = 30.0 * 30.0 * torque_params.steering.I_steer;
    const double expected_ki_effect = gains.inner_loop_ki_override * 0.001;

    EXPECT_NEAR(effective_stiffness, expected_kp + expected_ki_effect, 1e-6)
        << "Ki override should preserve auto-tuned Kp and only alter the integral term";
}

TEST_F(PIDDriverTorqueTest, SteeringActuatorLayerMatchesLegacyInnerLoopEquation) {
    SteeringActuatorConfig config;
    config.Kp_inner = 50.0;
    config.Ki_inner = 5.0;
    config.Kd_inner = 1.5;
    config.R_steer = 4.5;
    config.max_tire_angle = 0.35;
    config.max_driver_torque = 25.0;
    config.max_tire_steer_rate = 3.5;
    config.lag_lead_gain = 0.0;

    SteeringActuatorState actuator_state;
    const SteeringActuatorCommand command{
        0.10,  // target_tire_angle
        0.05,  // delta_sw
        0.20,  // ddelta_sw
        0.01,  // dt
    };

    const auto output = compute_steering_actuator_control(config, actuator_state, command);

    const double expected_target_sw = config.max_tire_steer_rate * config.R_steer * command.dt;
    const double expected_sw_error = expected_target_sw - command.delta_sw;
    const double expected_integral = expected_sw_error * command.dt;
    const double expected_raw =
        config.Kp_inner * expected_sw_error
      + config.Ki_inner * expected_integral
      - config.Kd_inner * command.ddelta_sw;

    EXPECT_NEAR(output.target_sw, expected_target_sw, 1e-12);
    EXPECT_NEAR(output.sw_error, expected_sw_error, 1e-12);
    EXPECT_NEAR(output.T_driver_raw, expected_raw, 1e-12);
    EXPECT_NEAR(output.T_driver_clamped, expected_raw, 1e-12);
    EXPECT_NEAR(output.steering_torque, expected_raw, 1e-12);
    EXPECT_NEAR(actuator_state.prev_target_sw, expected_target_sw, 1e-12);
    EXPECT_NEAR(actuator_state.inner_integral, expected_integral, 1e-12);
}

TEST_F(PIDDriverTorqueTest, SteeringActuatorLayerCancelsMeasuredSATDisturbance) {
    SteeringActuatorConfig config;
    config.Kp_inner = 50.0;
    config.Ki_inner = 5.0;
    config.Kd_inner = 1.5;
    config.R_steer = 4.5;
    config.max_driver_torque = 25.0;

    SteeringActuatorState actuator_state;
    SteeringActuatorCommand command;
    command.T_SAT_total = 18.0;

    const auto output = compute_steering_actuator_control(config, actuator_state, command);
    const double expected_feedforward = -command.T_SAT_total / config.R_steer;
    const double steering_acceleration = compute_steering_acceleration(
        0.02,
        output.steering_torque,
        command.T_SAT_total,
        config.R_steer,
        command.delta_sw,
        command.ddelta_sw,
        0.0,
        0.0);

    EXPECT_NEAR(output.T_SAT_feedforward, expected_feedforward, 1e-12);
    EXPECT_NEAR(output.T_driver_raw, expected_feedforward, 1e-12);
    EXPECT_NEAR(steering_acceleration, 0.0, 1e-12);
}

/// Test 7: 操舵トルク権限が低い車両では目標速度を自動的に下げる
TEST_F(PIDDriverTorqueTest, LowSteeringAuthorityReducesTargetSpeed) {
    VehicleParams nominal_params = VehicleParams::default_fsae();
    nominal_params.steering.torque_mode = true;
    nominal_params.steering.I_steer = 0.02;
    nominal_params.steering.T_fric = 0.3;
    nominal_params.steering.T_driver_max = 25.0;

    VehicleParams weak_params = nominal_params;
    weak_params.steering.T_driver_max = 12.0;

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    PIDPathFollowingDriver nominal_driver(track, velocity_profile, gains, nominal_params);
    nominal_driver.compute_control(state, 0.001);
    const double nominal_target = nominal_driver.get_last_debug().target_speed_final;

    PIDPathFollowingDriver weak_driver(track, velocity_profile, gains, weak_params);
    weak_driver.compute_control(state, 0.001);
    const double weak_target = weak_driver.get_last_debug().target_speed_final;

    EXPECT_LT(weak_target, nominal_target * 0.70)
        << "Low steering torque authority should lower target speed before off-track recovery";
}

/// Test 8: 低い操舵権限ではトルクモードの操舵previewを自動的に無効化する
TEST_F(PIDDriverTorqueTest, LowSteeringAuthorityDisablesSteeringPreview) {
    CurvatureBasedTrack circle_track = create_circle_track(8.0, 3.0);
    std::vector<VelocityPoint> profile;
    profile.push_back({0.0, 10.0});
    profile.push_back({circle_track.length(), 10.0});

    VehicleParams nominal_params = VehicleParams::default_fsae();
    nominal_params.steering.torque_mode = true;
    nominal_params.steering.I_steer = 0.02;
    nominal_params.steering.T_fric = 0.3;
    nominal_params.steering.T_driver_max = 25.0;

    VehicleParams weak_params = nominal_params;
    weak_params.steering.T_driver_max = 12.0;

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.steering_preview_distance = 10.0;
    gains.steering_preview_time = 0.0;
    gains.Kpp_steer = 0.0;
    gains.steering_preview_authority_threshold = 0.90;
    gains.line_planner.enabled = false;

    PIDGains no_preview_gains = gains;
    no_preview_gains.steering_preview_distance = 0.0;
    no_preview_gains.steering_preview_time = 0.0;

    auto start = circle_track.get_point(0.0);
    State14DOF state;
    state.x = start.y;
    state.y = start.x;
    state.psi = M_PI / 2.0 - start.theta;
    state.vx = 10.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver no_preview_driver(circle_track, profile, no_preview_gains, nominal_params);
    no_preview_driver.compute_control(state, 0.001);
    const double no_preview_delta = no_preview_driver.get_last_debug().delta_target;

    PIDPathFollowingDriver nominal_driver(circle_track, profile, gains, nominal_params);
    nominal_driver.compute_control(state, 0.001);
    const double nominal_delta = nominal_driver.get_last_debug().delta_target;

    PIDPathFollowingDriver weak_driver(circle_track, profile, gains, weak_params);
    weak_driver.compute_control(state, 0.001);
    const double weak_delta = weak_driver.get_last_debug().delta_target;

    EXPECT_GT(std::abs(nominal_delta - no_preview_delta), 0.05)
        << "Nominal authority should allow preview to influence steering target";
    EXPECT_NEAR(weak_delta, no_preview_delta, 1e-9)
        << "Low authority should fall back to no-preview steering target";
}

/// Test 9: トルクモードの lag lead は遅れている実舵に対して目標SW角を進める
TEST_F(PIDDriverTorqueTest, TorqueLagLeadAdvancesTargetSteeringWheel) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 100.0;
    torque_params.steering.R_steer = 4.5;

    PIDGains base_gains = conservative_gains();
    base_gains.torque_mode = true;
    base_gains.Kp_steer = 0.20;
    base_gains.Kd_steer = 0.0;
    base_gains.Ki_steer = 0.0;
    base_gains.Kh_steer = 0.0;
    base_gains.Kff_steer = 0.0;
    base_gains.max_tire_steer_rate = 1000.0;
    base_gains.max_steer_torque = 100.0;
    base_gains.torque_lag_lead_gain = 0.0;
    base_gains.torque_lag_lead_max_tire_angle = 1.0;

    State14DOF state;
    state.x = 0.10;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver no_lead_driver(track, velocity_profile, base_gains, torque_params);
    no_lead_driver.compute_control(state, 0.001);
    const auto no_lead_debug = no_lead_driver.get_last_debug();

    PIDGains lead_gains = base_gains;
    lead_gains.torque_lag_lead_gain = 0.50;

    PIDPathFollowingDriver lead_driver(track, velocity_profile, lead_gains, torque_params);
    lead_driver.compute_control(state, 0.001);
    const auto lead_debug = lead_driver.get_last_debug();

    ASSERT_GT(std::abs(no_lead_debug.target_sw), 1e-6);
    EXPECT_NEAR(no_lead_debug.torque_lag_lead_sw, 0.0, 1e-12);
    EXPECT_GT(std::abs(lead_debug.target_sw), std::abs(no_lead_debug.target_sw))
        << "Lag lead should command ahead of the nominal steering-wheel target";
    EXPECT_GT(std::abs(lead_debug.T_driver_raw), std::abs(no_lead_debug.T_driver_raw))
        << "Advancing the target should increase corrective torque when the wheel is lagging";
}

/// Test 10: lag lead 補正量は明示上限で制限される
TEST_F(PIDDriverTorqueTest, TorqueLagLeadCompensationIsLimited) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.R_steer = 4.5;

    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.max_tire_steer_rate = 1000.0;
    gains.torque_lag_lead_gain = 10.0;
    gains.torque_lag_lead_max_tire_angle = 0.01;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.20;
    state.psi = 0.0;
    state.vx = 10.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(track, velocity_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_LE(std::abs(debug.torque_lag_lead_sw), 0.01 * torque_params.steering.R_steer + 1e-12)
        << "Lead compensation should respect the tire-angle equivalent cap";
}

/// Test 11: 健全な直線区間ではトルクモード速度倍率を固定scaleから回復する
TEST_F(PIDDriverTorqueTest, TorqueSpeedRecoveryRaisesTargetOnHealthyStraight) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({100.0, 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 0.70;
    gains.torque_speed_recovery_gain = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.track_margin_speed_release = 0.5;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 15.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(track, fast_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_GT(debug.torque_speed_recovery_ratio, 0.95);
    EXPECT_NEAR(debug.target_speed_final, debug.target_speed_preview, 1e-9)
        << "Full recovery should restore the preview speed when steering health is good";
}

/// Test 12: 操舵遅れが大きい時は速度回復を許可しない
TEST_F(PIDDriverTorqueTest, TorqueSpeedRecoveryBlockedBySteeringLag) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({100.0, 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 0.70;
    gains.torque_speed_recovery_gain = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.torque_stability_vy_gain = 0.0;
    gains.torque_stability_yaw_gain = 0.0;
    gains.torque_stability_slip_gain = 0.0;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 15.0;
    state.delta_sw = 1.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(track, fast_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_NEAR(debug.torque_speed_recovery_ratio, 0.0, 1e-12);
    EXPECT_NEAR(debug.target_speed_final, debug.target_speed_preview * gains.torque_target_speed_scale, 1e-9)
        << "Large steering lag should keep the conservative torque speed scale";
}

/// Test 13: 直後に大きな操舵需要がある時は速度回復を許可しない
TEST_F(PIDDriverTorqueTest, TorqueSpeedRecoveryBlockedByPreviewSteeringDemand) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    CurvatureBasedTrack corner_track = create_straight_corner_track(30.0, 8.0, 4.5);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({corner_track.length(), 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 0.70;
    gains.torque_speed_recovery_gain = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 8.0;

    const auto pt = corner_track.get_point(26.0);
    State14DOF state;
    state.x = pt.y;
    state.y = pt.x;
    state.psi = M_PI / 2.0 - pt.theta;
    state.vx = 15.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(corner_track, fast_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    EXPECT_GT(std::abs(debug.preview_delta_gap), gains.torque_speed_recovery_preview_gap);
    EXPECT_NEAR(debug.torque_speed_recovery_ratio, 0.0, 1e-12)
        << "Recovery should stay disabled before a near-future steering demand";
}

TEST_F(PIDDriverTorqueTest, SteeringReachSpeedCapReducesTargetNearOutwardMargin) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_driver_max = 40.0;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 12.0});
    fast_profile.push_back({100.0, 12.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_recovery_gain = 0.0;
    gains.torque_speed_cap_preview_time = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.track_margin_preview_time = 0.50;
    gains.track_margin_speed_cap = 3.0;
    gains.track_margin_speed_release = 0.20;
    gains.torque_reach_speed_cap = 8.0;
    gains.torque_reach_time_start = 0.02;
    gains.torque_reach_time_full = 0.08;
    gains.torque_reach_outward_rate_start = 0.2;
    gains.torque_reach_outward_rate_full = 1.0;
    gains.torque_reach_min_preview_gap = 0.10;

    const auto pt = track.get_point(20.0);
    const double lateral_offset = 1.20;
    const double map_theta = pt.theta + 0.20;

    State14DOF state;
    state.y = pt.x - std::sin(pt.theta) * lateral_offset; // map_x -> vehicle y
    state.x = pt.y + std::cos(pt.theta) * lateral_offset; // map_y -> vehicle x
    state.psi = M_PI / 2.0 - map_theta;
    state.vx = 12.0;
    state.delta_sw = gains.max_steer * gains.R_steer;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(track, fast_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto debug = driver.get_last_debug();

    EXPECT_TRUE(std::isfinite(debug.steering_reach_speed_cap));
    EXPECT_LE(debug.steering_reach_speed_cap, gains.torque_reach_speed_cap + 1e-9);
    EXPECT_LE(debug.target_speed_final, gains.torque_reach_speed_cap + 1e-9);
    EXPECT_GT(debug.time_to_reach_delta, gains.torque_reach_time_start);
}

TEST_F(PIDDriverTorqueTest, TorqueSteeringPreviewDistanceScalesWithSpeed) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    CurvatureBasedTrack corner_track = create_straight_corner_track(30.0, 8.0, 4.5);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({corner_track.length(), 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_speed_recovery_gain = 0.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_time = 0.30;
    gains.torque_speed_cap_min_preview_distance = 0.0;
    gains.torque_speed_cap_max_preview_distance = 25.0;

    const auto pt = corner_track.get_point(25.0);
    State14DOF state;
    state.x = pt.y;
    state.y = pt.x;
    state.psi = M_PI / 2.0 - pt.theta;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    State14DOF low_speed_state = state;
    low_speed_state.vx = 10.0;  // 3m preview: corner at s=30 is outside the window
    PIDPathFollowingDriver low_speed_driver(corner_track, fast_profile, gains, torque_params);
    low_speed_driver.compute_control(low_speed_state, 0.001);
    const auto low_speed_debug = low_speed_driver.get_last_debug();

    State14DOF high_speed_state = state;
    high_speed_state.vx = 20.0;  // 6m preview: corner at s=30 is inside the window
    PIDPathFollowingDriver high_speed_driver(corner_track, fast_profile, gains, torque_params);
    high_speed_driver.compute_control(high_speed_state, 0.001);
    const auto high_speed_debug = high_speed_driver.get_last_debug();

    EXPECT_NEAR(low_speed_debug.required_delta_preview, 0.0, 1e-12);
    EXPECT_GT(std::abs(high_speed_debug.required_delta_preview), 0.05);
    EXPECT_GT(std::abs(high_speed_debug.preview_delta_gap),
              std::abs(low_speed_debug.preview_delta_gap));
}

TEST_F(PIDDriverTorqueTest, OuterLoopSteeringPreviewCanUseTimeBasedDistance) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    PIDGains gains = optimized_autocross_torque_gains();
    gains.steering_preview_distance = 20.0;
    gains.steering_preview_time = 0.50;
    gains.steering_preview_min_distance = 5.0;
    gains.steering_preview_max_distance = 12.0;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;

    State14DOF low_speed_state = state;
    low_speed_state.vx = 10.0;
    PIDPathFollowingDriver low_speed_driver(track, velocity_profile, gains, torque_params);
    low_speed_driver.compute_control(low_speed_state, 0.001);
    const auto low_speed_debug = low_speed_driver.get_last_debug();

    State14DOF high_speed_state = state;
    high_speed_state.vx = 30.0;
    PIDPathFollowingDriver high_speed_driver(track, velocity_profile, gains, torque_params);
    high_speed_driver.compute_control(high_speed_state, 0.001);
    const auto high_speed_debug = high_speed_driver.get_last_debug();

    EXPECT_NEAR(low_speed_debug.steering_preview_distance_used, 5.0, 1e-12);
    EXPECT_NEAR(high_speed_debug.steering_preview_distance_used, 12.0, 1e-12);
}

TEST_F(PIDDriverTorqueTest, RequiredPreviewTireAngleIncludesUndersteerDemand) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    const double radius = 20.0;
    const double speed = 15.0;
    CurvatureBasedTrack circle_track = create_circle_track(radius, 4.0);
    std::vector<VelocityPoint> circle_profile;
    circle_profile.push_back({0.0, speed});
    circle_profile.push_back({circle_track.length(), speed});

    PIDGains neutral_gains = optimized_autocross_torque_gains();
    neutral_gains.torque_target_speed_scale = 1.0;
    neutral_gains.torque_lag_speed_penalty = 0.0;
    neutral_gains.torque_saturation_speed_penalty = 0.0;
    neutral_gains.chassis_twist_speed_penalty = 0.0;
    neutral_gains.max_steer = 1.0;
    neutral_gains.understeer_gradient = 0.0;
    neutral_gains.torque_speed_cap_preview_time = 0.0;
    neutral_gains.torque_speed_cap_preview_distance = 0.0;

    PIDGains understeer_gains = neutral_gains;
    understeer_gains.understeer_gradient = 0.01;

    const auto pt = circle_track.get_point(0.0);
    State14DOF state;
    state.x = pt.y;
    state.y = pt.x;
    state.psi = M_PI / 2.0 - pt.theta;
    state.vx = speed;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver neutral_driver(circle_track, circle_profile, neutral_gains, torque_params);
    neutral_driver.compute_control(state, 0.001);
    const auto neutral_debug = neutral_driver.get_last_debug();

    PIDPathFollowingDriver understeer_driver(circle_track, circle_profile, understeer_gains, torque_params);
    understeer_driver.compute_control(state, 0.001);
    const auto understeer_debug = understeer_driver.get_last_debug();

    EXPECT_GT(std::abs(understeer_debug.required_delta_preview),
              std::abs(neutral_debug.required_delta_preview));
    EXPECT_NEAR(std::abs(neutral_debug.required_delta_preview),
                neutral_gains.wheelbase / radius,
                1e-6);
}

/// Test 14: トルクモードのDRS判定は操舵トルクではなく実/目標舵角で行う
TEST_F(PIDDriverTorqueTest, TorqueModeDrsUsesSteerAngleNotTorqueCommand) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 25.0});
    fast_profile.push_back({100.0, 25.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.Kp_steer = 0.02;
    gains.Kd_steer = 0.0;
    gains.Ki_steer = 0.0;
    gains.Kh_steer = 0.0;
    gains.Kff_steer = 0.0;
    gains.Kpp_steer = 0.0;
    gains.torque_target_speed_scale = 1.0;
    gains.max_tire_steer_rate = 1000.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.Kp_steer_inner = 50.0;
    gains.Ki_steer_inner = 0.0;
    gains.Kd_steer_inner = 0.0;
    gains.inner_loop_auto_tune = false;

    State14DOF state;
    state.x = 0.10;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 25.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver(track, fast_profile, gains, torque_params);
    const auto input = driver.compute_control(state, 0.001);
    const auto& debug = driver.get_last_debug();

    ASSERT_GT(std::abs(input.steering), 0.05)
        << "This test requires a nonzero torque command that would fail the old DRS check";
    EXPECT_LT(std::max(std::abs(debug.actual_delta_tire), std::abs(debug.delta_target)), 0.05);
    EXPECT_TRUE(input.drs_button)
        << "Torque-mode DRS should be allowed on a straight when steer angles are small";
}

/// Test 15: トルクモードのコーナースロットル制限は実タイヤ舵角を指標として使う
TEST_F(PIDDriverTorqueTest, TorqueAutocrossPresetKeepsCornerThrottleLimiterMild) {
    PIDGains gains = optimized_autocross_torque_gains();

    EXPECT_TRUE(gains.torque_mode);
    EXPECT_GT(gains.Kpp_steer, 0.0)
        << "Torque autocross should include geometric preview steering assist.";
    EXPECT_DOUBLE_EQ(gains.steering_preview_time, 0.0)
        << "Time-based preview is available but should stay opt-in until validated faster.";
    EXPECT_GT(gains.corner_throttle_reduction, 0.0);
    EXPECT_LE(gains.corner_throttle_reduction, 0.20)
        << "Torque-mode steering lag and track-margin speed limits should carry "
        << "the primary entry-speed control; post-PI throttle clipping must remain mild.";
}

TEST_F(PIDDriverTorqueTest, TorqueModeCornerThrottleUsesActualTireSteerForProtection) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 25.0});
    fast_profile.push_back({100.0, 25.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.Kp_steer = 0.02;
    gains.Kd_steer = 0.0;
    gains.Ki_steer = 0.0;
    gains.Kh_steer = 0.0;
    gains.Kff_steer = 0.0;
    gains.Kpp_steer = 0.0;
    gains.torque_target_speed_scale = 1.0;
    gains.max_tire_steer_rate = 1000.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.Kp_steer_inner = 50.0;
    gains.Ki_steer_inner = 0.0;
    gains.Kd_steer_inner = 0.0;
    gains.inner_loop_auto_tune = false;
    gains.corner_throttle_reduction = 0.6; // 60% max reduction

    State14DOF state_straight;
    state_straight.x = 0.0;
    state_straight.y = 0.0;
    state_straight.psi = M_PI / 2.0;
    state_straight.vx = 20.0;
    state_straight.delta_sw = 0.0;
    state_straight.ddelta_sw = 0.0;

    PIDPathFollowingDriver driver_straight(track, fast_profile, gains, torque_params);
    driver_straight.compute_control(state_straight, 0.001);
    const auto& debug_straight = driver_straight.get_last_debug();

    // 舵角が0のときはスロットル制限がかからないこと
    EXPECT_DOUBLE_EQ(debug_straight.corner_throttle_steer_ratio, 0.0);
    EXPECT_DOUBLE_EQ(debug_straight.corner_throttle_scale, 1.0);

    // 最大舵角の半分のステアリング角を設定
    State14DOF state_turned = state_straight;
    state_turned.delta_sw = 0.5 * gains.max_steer * gains.R_steer;

    PIDPathFollowingDriver driver_turned(track, fast_profile, gains, torque_params);
    driver_turned.compute_control(state_turned, 0.001);
    const auto& debug_turned = driver_turned.get_last_debug();

    // 舵角比率が0.5、スロットル制限が1.0 - 0.5*0.6 = 0.7になること
    EXPECT_NEAR(debug_turned.corner_throttle_steer_ratio, 0.5, 1e-6);
    EXPECT_NEAR(debug_turned.corner_throttle_limit_ratio, 0.5, 1e-6);
    EXPECT_NEAR(debug_turned.corner_throttle_scale, 0.7, 1e-6);
}

TEST_F(PIDDriverTorqueTest, CornerThrottleUsesMuAwareLateralUtilization) {
    CurvatureBasedTrack circle_track = create_circle_track(20.0, 4.0);
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 16.0});
    fast_profile.push_back({circle_track.length(), 16.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.corner_throttle_reduction = 0.6;
    gains.corner_throttle_lateral_utilization_start = 0.75;
    gains.corner_throttle_lateral_utilization_full = 1.15;
    gains.corner_throttle_drive_slip_start = 0.08;
    gains.corner_throttle_drive_slip_full = 0.20;

    const auto pt = circle_track.get_point(0.0);
    State14DOF state;
    state.x = pt.y;
    state.y = pt.x;
    state.psi = M_PI / 2.0 - pt.theta;
    state.vx = 14.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;
    for (double& omega : state.omega) {
        omega = state.vx / VehicleParams::default_fsae().tire.radius;
    }

    VehicleParams high_mu_params = VehicleParams::default_fsae();
    high_mu_params.steering.torque_mode = true;
    high_mu_params.tire.mu_y_max = 2.0;

    VehicleParams low_mu_params = high_mu_params;
    low_mu_params.tire.mu_y_max = 1.0;

    PIDPathFollowingDriver high_mu_driver(circle_track, fast_profile, gains, high_mu_params);
    high_mu_driver.compute_control(state, 0.001);
    const auto high_mu_debug = high_mu_driver.get_last_debug();

    PIDPathFollowingDriver low_mu_driver(circle_track, fast_profile, gains, low_mu_params);
    low_mu_driver.compute_control(state, 0.001);
    const auto low_mu_debug = low_mu_driver.get_last_debug();

    EXPECT_LT(high_mu_debug.corner_throttle_lateral_ratio,
              low_mu_debug.corner_throttle_lateral_ratio);
    EXPECT_LT(low_mu_debug.corner_throttle_scale, high_mu_debug.corner_throttle_scale);
    EXPECT_GT(low_mu_debug.corner_throttle_limit_ratio,
              low_mu_debug.corner_throttle_steer_ratio);
}

TEST_F(PIDDriverTorqueTest, CornerThrottleUsesRearDriveSlip) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 25.0});
    fast_profile.push_back({100.0, 25.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.corner_throttle_reduction = 0.5;
    gains.corner_throttle_lateral_utilization_start = 1.5;
    gains.corner_throttle_lateral_utilization_full = 2.0;
    gains.corner_throttle_drive_slip_start = 0.08;
    gains.corner_throttle_drive_slip_full = 0.20;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 20.0;
    state.delta_sw = 0.0;
    state.ddelta_sw = 0.0;
    for (double& omega : state.omega) {
        omega = state.vx / torque_params.tire.radius;
    }
    state.omega[RL] = 1.15 * state.vx / torque_params.tire.radius;
    state.omega[RR] = 1.15 * state.vx / torque_params.tire.radius;

    PIDPathFollowingDriver driver(track, fast_profile, gains, torque_params);
    driver.compute_control(state, 0.001);
    const auto debug = driver.get_last_debug();

    EXPECT_GT(debug.corner_throttle_drive_slip, gains.corner_throttle_drive_slip_start);
    EXPECT_GT(debug.corner_throttle_drive_slip_ratio, 0.0);
    EXPECT_NEAR(debug.corner_throttle_steer_ratio, 0.0, 1e-12);
    EXPECT_NEAR(debug.corner_throttle_limit_ratio,
                debug.corner_throttle_drive_slip_ratio,
                1e-12);
    EXPECT_LT(debug.corner_throttle_scale, 1.0);
}

/// Test 16: コーナースロットル制限はトルクモードの有無に関わらずタイヤ実舵角ベースで動作する
TEST_F(PIDDriverTorqueTest, CornerThrottleLimitsThrottleBasedOnSteerAngleRegardlessOfTorqueMode) {
    VehicleParams params = VehicleParams::default_fsae();
    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 25.0});
    fast_profile.push_back({100.0, 25.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.max_tire_steer_rate = 1000.0;
    gains.torque_speed_cap_preview_distance = 0.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.corner_throttle_reduction = 0.5;

    // A. トルクモードでの検証
    VehicleParams torque_params = params;
    torque_params.steering.torque_mode = true;
    PIDGains torque_gains = gains;
    torque_gains.torque_mode = true;
    torque_gains.inner_loop_auto_tune = false;

    State14DOF state_torque;
    state_torque.x = 0.0;
    state_torque.y = 0.0;
    state_torque.psi = M_PI / 2.0;
    state_torque.vx = 20.0;
    state_torque.delta_sw = 0.8 * gains.max_steer * gains.R_steer; // 80% max steer angle

    PIDPathFollowingDriver driver_torque(track, fast_profile, torque_gains, torque_params);
    driver_torque.compute_control(state_torque, 0.001);
    const auto& debug_torque = driver_torque.get_last_debug();

    // B. 角度モードでの検証
    VehicleParams angle_params = params;
    angle_params.steering.torque_mode = false;
    PIDGains angle_gains = gains;
    angle_gains.torque_mode = false;

    // 角度モードでは、目標タイヤ舵角（delta_target）が 0.8 * gains.max_steer になるように、
    // 進路偏差を設定する。目標軌道がy=0に対して、車両の横偏差を調整する。
    // pid_driver.hppの横偏差による舵角は Kp_steer * error_n などで決まる。
    // delta_target = Kp_steer * error_n + Kff * ff_steer + ...
    // ここでは、delta_targetが 1.0 * gains.max_steer (クランプされる領域) になるように、
    // 横偏差 (state_angle.x) とヘディング誤差 (state_angle.psi = 0) を大きく設定して飽和させる。
    State14DOF state_angle;
    state_angle.x = 10.0; // 横偏差 10.0 m (Map Y = 10.0)
    state_angle.y = 0.0;  // Map X = 0.0
    state_angle.psi = 0.0; // ヘディング誤差 90度 (Map Theta = M_PI / 2.0)
    state_angle.vx = 20.0;
    state_angle.delta_sw = 0.0;

    PIDPathFollowingDriver driver_angle(track, fast_profile, angle_gains, angle_params);
    driver_angle.compute_control(state_angle, 0.001);
    const auto& debug_angle = driver_angle.get_last_debug();

    // 両方のモードで舵角比率が 1.0 (最大クランプ) もしくは 0.8 (正確な目標) になり、
    // corner_throttle_scale が物理的に一致することを確認。
    // 角度モードでは delta_target が max_steer にクランプされるため、比率は 1.0 になるはず。
    // トルクモード側も delta_sw が 1.0 * gains.max_steer * gains.R_steer になるように合わせて比較する。
    
    // トルクモード側を max_steer で設定しなおす
    state_torque.delta_sw = gains.max_steer * gains.R_steer;
    PIDPathFollowingDriver driver_torque_saturated(track, fast_profile, torque_gains, torque_params);
    driver_torque_saturated.compute_control(state_torque, 0.001);
    const auto& debug_torque_saturated = driver_torque_saturated.get_last_debug();

    EXPECT_NEAR(debug_torque_saturated.corner_throttle_steer_ratio, 1.0, 1e-6);
    EXPECT_NEAR(debug_angle.corner_throttle_steer_ratio, 1.0, 1e-6);
    EXPECT_DOUBLE_EQ(debug_torque_saturated.corner_throttle_scale, debug_angle.corner_throttle_scale);
}

/// Test 17: トルク権限が高い車両では目標タイヤ舵角レートを自動的に引き上げる
TEST_F(PIDDriverTorqueTest, SteeringRateLimitScalesWithTorqueAuthority) {
    VehicleParams nominal_params = VehicleParams::default_fsae();
    nominal_params.steering.torque_mode = true;
    nominal_params.steering.I_steer = 0.02;
    nominal_params.steering.T_fric = 0.3;
    nominal_params.steering.T_driver_max = 25.0;

    VehicleParams high_torque_params = nominal_params;
    high_torque_params.steering.T_driver_max = 40.0;

    VehicleParams high_inertia_params = high_torque_params;
    high_inertia_params.steering.I_steer = 0.04;

    std::vector<VelocityPoint> fast_profile;
    fast_profile.push_back({0.0, 20.0});
    fast_profile.push_back({100.0, 20.0});

    PIDGains gains = optimized_autocross_torque_gains();
    gains.max_tire_steer_rate = 3.0;
    gains.torque_steer_rate_min_scale = 0.65;
    gains.torque_steer_rate_max_scale = 1.35;
    gains.torque_steer_rate_authority_exponent = 0.5;

    State14DOF state;
    state.x = 0.0;
    state.y = 0.0;
    state.psi = M_PI / 2.0;
    state.vx = 10.0;

    PIDPathFollowingDriver nominal_driver(track, fast_profile, gains, nominal_params);
    nominal_driver.compute_control(state, 0.001);
    const auto& nominal_debug = nominal_driver.get_last_debug();

    PIDPathFollowingDriver high_torque_driver(track, fast_profile, gains, high_torque_params);
    high_torque_driver.compute_control(state, 0.001);
    const auto& high_torque_debug = high_torque_driver.get_last_debug();

    PIDPathFollowingDriver high_inertia_driver(track, fast_profile, gains, high_inertia_params);
    high_inertia_driver.compute_control(state, 0.001);
    const auto& high_inertia_debug = high_inertia_driver.get_last_debug();

    const double expected_high_torque_rate =
        3.0 * std::sqrt((40.0 - 0.3) / (25.0 - 0.3));
    const double expected_high_inertia_rate =
        3.0 * std::sqrt(((40.0 - 0.3) / 0.04) / ((25.0 - 0.3) / 0.02));

    EXPECT_NEAR(nominal_debug.effective_max_tire_steer_rate, 3.0, 1e-12);
    EXPECT_NEAR(high_torque_debug.effective_max_tire_steer_rate, expected_high_torque_rate, 1e-12);
    EXPECT_NEAR(high_inertia_debug.effective_max_tire_steer_rate, expected_high_inertia_rate, 1e-12);
    EXPECT_GT(high_torque_debug.effective_max_tire_steer_rate,
              nominal_debug.effective_max_tire_steer_rate);
    EXPECT_LT(high_inertia_debug.effective_max_tire_steer_rate,
              high_torque_debug.effective_max_tire_steer_rate);
}

/// Test 18: Stability penalties reduce target speed multiplier based on lateral dynamics and slip limits
TEST_F(PIDDriverTorqueTest, StabilityPenaltiesReduceTargetSpeed) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;
    torque_params.steering.I_steer = 0.02;
    torque_params.steering.T_fric = 0.3;
    torque_params.steering.T_driver_max = 25.0;

    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_speed_cap_preview_distance = 0.0;

    // Stability configurations
    gains.torque_stability_vy_threshold = 1.0;     // 1.0 m/s
    gains.torque_stability_vy_gain = 0.20;         // 0.20 s/m
    gains.torque_stability_yaw_threshold = 0.20;   // 0.20 rad/s
    gains.torque_stability_yaw_gain = 0.50;        // 0.50 s/rad
    gains.torque_stability_slip_threshold = 0.08;  // ~4.5 deg
    gains.torque_stability_slip_gain = 2.0;        // 2.0 / rad
    gains.distance_cg_front = 0.73;

    // 1. Baseline state (highly stable)
    PIDPathFollowingDriver driver_base(track, velocity_profile, gains, torque_params);
    State14DOF base_state;
    base_state.x = 0.0;
    base_state.y = 0.0;
    base_state.psi = 0.0;
    base_state.vx = 10.0;
    base_state.vy = 0.0;
    base_state.r = 0.0;
    base_state.delta_sw = 0.0;

    driver_base.compute_control(base_state, 0.001);
    auto debug_base = driver_base.get_last_debug();
    EXPECT_DOUBLE_EQ(debug_base.stability_vy_penalty, 0.0);
    EXPECT_DOUBLE_EQ(debug_base.stability_yaw_penalty, 0.0);
    EXPECT_DOUBLE_EQ(debug_base.stability_slip_penalty, 0.0);
    EXPECT_DOUBLE_EQ(debug_base.stability_penalty, 0.0);
    EXPECT_DOUBLE_EQ(debug_base.torque_speed_multiplier, 1.0);

    // 2. Excess lateral velocity (vy = 2.0 m/s). Isolate vy gain by setting slip_gain to 0.0
    PIDGains gains_vy = gains;
    gains_vy.torque_stability_slip_gain = 0.0;
    gains_vy.torque_stability_yaw_gain = 0.0;
    PIDPathFollowingDriver driver_vy(track, velocity_profile, gains_vy, torque_params);
    State14DOF vy_state = base_state;
    vy_state.vy = 2.0; // 1.0 m/s excess over 1.0 threshold -> 1.0 * 0.2 = 0.2 penalty
    driver_vy.compute_control(vy_state, 0.001);
    auto debug_vy = driver_vy.get_last_debug();
    EXPECT_NEAR(debug_vy.stability_vy_penalty, 0.2, 1e-5);
    EXPECT_NEAR(debug_vy.stability_penalty, 0.2, 1e-5);
    EXPECT_NEAR(debug_vy.torque_speed_multiplier, 0.8, 1e-5);

    // 3. Yaw rate error. Isolate yaw gain by setting slip_gain to 0.0
    PIDGains gains_yaw = gains;
    gains_yaw.torque_stability_vy_gain = 0.0;
    gains_yaw.torque_stability_slip_gain = 0.0;
    PIDPathFollowingDriver driver_yaw(track, velocity_profile, gains_yaw, torque_params);
    State14DOF yaw_state = base_state;
    yaw_state.r = 0.6; // excess r: 0.6 - 0.2 = 0.4 -> penalty: 0.4 * 0.5 = 0.2
    driver_yaw.compute_control(yaw_state, 0.001);
    auto debug_yaw = driver_yaw.get_last_debug();
    EXPECT_NEAR(debug_yaw.stability_yaw_penalty, 0.2, 1e-5);
    EXPECT_NEAR(debug_yaw.stability_penalty, 0.2, 1e-5);
    EXPECT_NEAR(debug_yaw.torque_speed_multiplier, 0.8, 1e-5);

    // 4. Front slip angle excess. Isolate slip gain
    PIDGains gains_slip = gains;
    gains_slip.torque_stability_vy_gain = 0.0;
    gains_slip.torque_stability_yaw_gain = 0.0;
    PIDPathFollowingDriver driver_slip(track, velocity_profile, gains_slip, torque_params);
    State14DOF slip_state = base_state;
    slip_state.delta_sw = 0.20 * 4.5; // using R_steer = 4.5 -> actual_delta_tire = 0.20 rad
    driver_slip.compute_control(slip_state, 0.001);
    auto debug_slip = driver_slip.get_last_debug();
    EXPECT_NEAR(debug_slip.estimated_front_slip, 0.20, 1e-3);
    EXPECT_NEAR(debug_slip.stability_slip_penalty, 0.24, 1e-3);
    EXPECT_NEAR(debug_slip.stability_penalty, 0.24, 1e-3);
    EXPECT_NEAR(debug_slip.torque_speed_multiplier, 0.76, 1e-3);
}

/// Test 19: Track curvature sign is converted to the vehicle yaw-rate convention
TEST_F(PIDDriverTorqueTest, StabilityYawReferenceUsesVehicleYawSign) {
    VehicleParams torque_params = VehicleParams::default_fsae();
    torque_params.steering.torque_mode = true;

    PIDGains gains = conservative_gains();
    gains.torque_mode = true;
    gains.torque_target_speed_scale = 1.0;
    gains.torque_lag_speed_penalty = 0.0;
    gains.torque_saturation_speed_penalty = 0.0;
    gains.chassis_twist_speed_penalty = 0.0;
    gains.torque_stability_vy_gain = 0.0;
    gains.torque_stability_slip_gain = 0.0;
    gains.torque_stability_yaw_threshold = 0.05;
    gains.torque_stability_yaw_gain = 1.0;
    gains.torque_speed_cap_preview_time = 0.0;
    gains.torque_speed_cap_min_preview_distance = 0.0;
    gains.torque_speed_cap_max_preview_distance = 0.0;

    const double radius = 10.0;
    const double speed = 8.0;
    const double expected_yaw_rate = speed / radius;
    std::vector<VelocityPoint> circle_profile = {
        {0.0, speed},
        {2.0 * M_PI * radius, speed},
    };

    State14DOF state;
    state.x = 0.0;       // map_y at the analytical circle start
    state.y = radius;    // map_x at the analytical circle start
    state.vx = speed;
    state.vy = 0.0;
    state.delta_sw = 0.0;

    CurvatureBasedTrack left_track = create_circle_track(radius, 3.0);
    state.psi = 0.0;                 // map_theta = pi/2
    state.r = -expected_yaw_rate;    // left turn is negative in vehicle yaw convention
    PIDPathFollowingDriver left_driver(left_track, circle_profile, gains, torque_params);
    left_driver.compute_control(state, 0.001);
    const auto left_debug = left_driver.get_last_debug();
    EXPECT_NEAR(left_debug.kappa, 1.0 / radius, 1e-12);
    EXPECT_NEAR(left_debug.stability_yaw_penalty, 0.0, 1e-12);

    state.r = expected_yaw_rate;
    PIDPathFollowingDriver left_wrong_driver(left_track, circle_profile, gains, torque_params);
    left_wrong_driver.compute_control(state, 0.001);
    const auto left_wrong_debug = left_wrong_driver.get_last_debug();
    EXPECT_GT(left_wrong_debug.stability_yaw_penalty, 1.0);

    CurvatureBasedTrack right_track = create_circle_cw_track(radius, 3.0);
    state.psi = M_PI;                // map_theta = -pi/2
    state.r = expected_yaw_rate;     // right turn is positive in vehicle yaw convention
    PIDPathFollowingDriver right_driver(right_track, circle_profile, gains, torque_params);
    right_driver.compute_control(state, 0.001);
    const auto right_debug = right_driver.get_last_debug();
    EXPECT_NEAR(right_debug.kappa, -1.0 / radius, 1e-12);
    EXPECT_NEAR(right_debug.stability_yaw_penalty, 0.0, 1e-12);

    state.r = -expected_yaw_rate;
    PIDPathFollowingDriver right_wrong_driver(right_track, circle_profile, gains, torque_params);
    right_wrong_driver.compute_control(state, 0.001);
    const auto right_wrong_debug = right_wrong_driver.get_last_debug();
    EXPECT_GT(right_wrong_debug.stability_yaw_penalty, 1.0);
}

/// Test 20: Optimized torque autocross preset does not rely on false yaw penalties
TEST_F(PIDDriverTorqueTest, OptimizedTorquePresetUsesRelaxedStabilityAndMarginPreview) {
    PIDGains gains = optimized_autocross_torque_gains();

    EXPECT_TRUE(gains.torque_mode);
    EXPECT_NEAR(gains.Kp_steer, 0.70, 1e-12);
    EXPECT_NEAR(gains.Kff_steer, 1.60, 1e-12);
    EXPECT_NEAR(gains.torque_target_speed_scale, 0.90, 1e-12);
    EXPECT_NEAR(gains.max_tire_steer_rate, 3.5, 1e-12);
    EXPECT_NEAR(gains.target_speed_preview_distance, 6.0, 1e-12);
    EXPECT_NEAR(gains.track_margin_preview_time, 0.50, 1e-12);
    EXPECT_NEAR(gains.track_margin_speed_release, 1.0, 1e-12);
    EXPECT_NEAR(gains.torque_stability_yaw_threshold, 0.30, 1e-12);
    EXPECT_NEAR(gains.torque_stability_yaw_gain, 0.30, 1e-12);
    EXPECT_NEAR(gains.torque_stability_slip_threshold, 0.12, 1e-12);
    EXPECT_NEAR(gains.torque_stability_slip_gain, 1.20, 1e-12);
    EXPECT_NEAR(gains.torque_reach_speed_cap, 10.0, 1e-12);
    EXPECT_NEAR(gains.torque_reach_time_start, 0.13, 1e-12);
    EXPECT_NEAR(gains.torque_reach_time_full, 0.22, 1e-12);
    EXPECT_TRUE(gains.line_planner.enabled);
    EXPECT_NEAR(gains.line_planner.offset_fraction, 0.05, 1e-12);
    EXPECT_NEAR(gains.line_planner.safety_margin, 0.55, 1e-12);
    EXPECT_NEAR(gains.line_planner.preview_distance, 10.0, 1e-12);
    EXPECT_FALSE(gains.line_planner.track_geometry);
}
