/// @file main_laptime_sim.cpp
/// @brief フル過渡ラップタイムシミュレーション
///
/// 総合ガイド §15.4 に基づく実装
/// トラック・速度プロファイル・ドライバー・車両を統合した完全なラップシミュレーション

#include <fsae/driver/pid_driver.hpp>
#include <fsae/driver/attack_driver.hpp>
#include <fsae/driver/driver_capability_profile.hpp>
#include <fsae/driver/mpc_driver.hpp>
#include <fsae/driver/mpc_driver_v2.hpp>
#include <fsae/driver/lpv_lqr_driver.hpp>
#include <fsae/driver/rl_driver.hpp>
#include <fsae/driver/state_frame.hpp>
#include <fsae/vehicle/fast_lap_vehicle.hpp>
#include <fsae/vehicle/lap_vehicle.hpp>
#include <fsae/track/track_model.hpp>
#include <fsae/track/waypoint_loader.hpp> // Added for CSV loading
#include <fsae/laptime/velocity_profile.hpp>
#include <fsae/laptime/velocity_profile_io.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <utility>
#include <vector>
#include <array>
#include <limits>
#include <optional>

using namespace fsae;
using json = nlohmann::json;

/// シミュレーション設定
struct SimulationConfig {
    std::string track_name = "circle";
    double track_radius = 30.0;        // [m]
    double track_width = 3.0;          // [m]
    double simulation_time = 30.0;     // [s]
    bool simulation_time_explicit = false;
    double dt = 0.001;                 // [s] (1ms)
    bool dt_explicit = false;
    double output_rate = 0.01;         // [s] (10ms毎に出力)
    int laps = 1;                      // score-equivalent 用 multi-lap
    std::string eval_mode = "full";    // "full" or "fast"
    bool allow_defaults = false;       // test/exploration: skip strict artifact validation (#2)

    // 車両性能パラメータ
    double max_lat_accel = 1.5 * 9.81;
    bool max_lat_accel_explicit = false;
    double max_lon_accel = 1.0 * 9.81;
    double max_lon_decel = 1.8 * 9.81;
    double max_speed = 50.0;

    // 初期速度（Phase 7改善v2: さらに安定性重視）
    double initial_speed = 6.0;        // [m/s] ローリングスタート既定（acceleration は 0 = 静止発進, Issue #24）
    bool initial_speed_explicit = false;

    // PIDゲインプリセット
    std::string gain_preset = "default";  // "conservative", "default", "aggressive"
    std::string driver_mode = "pid";      // "pid", "lpv_lqr", "mpc", "mpc_v2", "attack", or "rl"
    std::string rl_actor_dir = "";        // required when driver_mode == "rl"
    double rl_steer_torque_rate_limit = -1.0; // [Nm/s], <0 uses vehicle contract
    int mpc_horizon = -1;                  // preview steps; <0 uses track-specific default
    double mpc_dt = -1.0;                  // [s], MPC preview/control step; <0 uses default
    std::string lpv_model = "steering6";   // "bicycle4" or "steering6"
    double lpv_lateral_weight_scale = 1.0;
    double lpv_heading_weight_scale = 1.0;
    double attack_speed_lateral_safety = -1.0;
    double attack_speed_braking_safety = -1.0;
    double attack_speed_accel_limit = -1.0;
    double attack_rollout_understeer_gradient = -1.0;
    double attack_floor_straight = -1.0;
    double attack_floor_exit = -1.0;
    double attack_floor_complex = -1.0;
    double attack_floor_turn_in = -1.0;
    double attack_floor_apex = -1.0;
    double attack_floor_brake = -1.0;
    double attack_braking_corridor_decel_scale = -1.0;
    double attack_braking_corridor_band_scale = -1.0;
    double attack_braking_corridor_mild_accel_scale = -1.0;
    double attack_braking_corridor_overspeed_brake_gain = -1.0;
    bool attack_braking_corridor_guard_only = true;
    double attack_w_y = -1.0;
    double attack_w_psi = -1.0;
    double attack_w_vx = -1.0;
    double attack_w_delta = -1.0;
    double attack_w_ax = -1.0;
    double attack_w_delta_rate = -1.0;
    double attack_w_ax_rate = -1.0;
    double attack_terminal_progress_weight = -1.0;
    double attack_progress_target_fraction = -1.0;
    double attack_soft_boundary_weight = -1.0;
    double attack_outward_rate_weight = -1.0;
    std::string attack_speed_cap_file = "";
    std::string attack_residual_profile_file = "";
    std::string driver_capability_profile_file = "";

    // 個別PIDゲイン（指定された場合はプリセットを上書き）
    bool use_custom_gains = false;
    double Kp_steer = -1.0;
    double Kd_steer = -1.0;
    double Ki_steer = -1.0;
    double Kh_steer = -1.0;
    double Kff_steer = -1.0;
    double Kp_speed = -1.0;
    double Ki_speed = -1.0;
    double max_throttle = -1.0;
    double max_steer = -1.0;
    double brake_gain_ratio = -1.0;        // ブレーキ感度係数
    double corner_throttle_reduction = -1.0; // コーナースロットル減少率
    double corner_throttle_torque_reference = -1.0;
    double corner_throttle_lateral_utilization_start = -1.0;
    double corner_throttle_lateral_utilization_full = -1.0;
    double corner_throttle_drive_slip_start = -1.0;
    double corner_throttle_drive_slip_full = -1.0;
    double target_speed_preview_distance = -1.0;
    double torque_speed_scale = -1.0;
    double torque_preview_distance = -1.0;
    double torque_speed_cap_preview_distance = -1.0;
    double torque_outer_loop_scale = -1.0;
    double steering_driver_torque_max = -1.0;
    double steering_driver_torque_rate_max = -1.0;
    double max_tire_steer_rate = -1.0;
    double torque_steer_rate_reference_torque = -1.0;
    double torque_steer_rate_reference_inertia = -1.0;
    double torque_steer_rate_authority_exponent = -1.0;
    double torque_steer_rate_min_scale = -1.0;
    double torque_steer_rate_max_scale = -1.0;
    double torque_lag_lead_gain = -1.0;
    double torque_lag_lead_time = -1.0;
    double torque_lag_lead_max_tire_angle = -1.0;
    double torque_speed_recovery_gain = -1.0;
    double torque_speed_recovery_curvature = -1.0;
    double torque_speed_recovery_lag_ratio = -1.0;
    double torque_speed_recovery_lateral_error = -1.0;
    double torque_speed_recovery_heading_error = -1.0;
    double torque_speed_recovery_preview_gap = -1.0;
    double torque_speed_recovery_reach_time = -1.0;
    double torque_reach_speed_cap = -1.0;
    double torque_reach_time_start = -1.0;
    double torque_reach_time_full = -1.0;
    double torque_reach_outward_rate_start = -1.0;
    double torque_reach_outward_rate_full = -1.0;
    double torque_reach_min_preview_gap = -1.0;
    double track_margin_speed_cap = -1.0;
    double track_margin_speed_release = -1.0;
    double track_margin_preview_time = -1.0;
    double steer_inner_ki = -1.0;
    double steer_inner_omega = -1.0;
    double steer_inner_zeta = -1.0;
    double steering_preview_distance = -1.0;
    double steering_preview_time = -1.0;
    double steering_preview_min_distance = -1.0;
    double steering_preview_max_distance = -1.0;
    double kpp_steer = -1.0;
    double understeer_gradient = -1.0;
    double torque_speed_cap_preview_time = -1.0;
    double torque_speed_cap_min_preview_distance = -1.0;
    double torque_speed_cap_max_preview_distance = -1.0;
    double torque_stability_vy_threshold = -1.0;
    double torque_stability_vy_gain = -1.0;
    double torque_stability_yaw_threshold = -1.0;
    double torque_stability_yaw_gain = -1.0;
    double torque_stability_slip_threshold = -1.0;
    double torque_stability_slip_gain = -1.0;
    double distance_cg_front = -1.0;
    bool line_planner_enabled = false;
    bool line_planner_disabled = false;
    double line_offset_fraction = -1.0;
    double line_safety_margin = -1.0;
    double line_preview_distance = -1.0;
    double line_smoothing_distance = -1.0;
    double line_scan_step = -1.0;
    double line_entry_offset_scale = -1.0;
    double line_apex_offset_scale = -1.0;
    double line_exit_offset_scale = -1.0;
    bool line_track_geometry_enabled = false;
    bool line_track_geometry_disabled = false;
    bool line_profile_curvature = true;
    
    // パラメータ注入
    std::string params_file = "";
    std::string params_json_str = "";
    bool output_json_result = false;
    std::string output_file = ""; // Explicit output filename
    std::string rl_teacher_output_file = "";
    std::string velocity_profile_input_file = "";
    std::string velocity_profile_output_file = "";

    // 単一摩擦利用マージン（Issue #26, --friction-margin）
    double friction_margin = laptime::kDefaultFrictionMargin;

    // オフライン GGV（Issue #23, --ggv）。空なら閉形式プランナー。--max-lon-* は GGV 時は上限のみ
    std::string ggv_file;
    bool max_lon_explicit = false;

    // 風ベクトル（地図座標 [m/s]、Issue #28, --wind wx,wy）。プラントへの受け渡しは WIRING.md
    double wind_x = 0.0;
    double wind_y = 0.0;

    // CSV トラックの周回フラグ（--track-closed / --track-open）。未指定は CSV `# closed=` → 警告付きヒューリスティック
    std::optional<bool> track_closed;
};

bool is_autocross_like_track(const SimulationConfig& config) {
    return config.track_name == "autocross" ||
           config.track_name == "ASE_autocross" ||
           (config.track_name.length() > 4 &&
            config.track_name.substr(config.track_name.length() - 4) == ".csv");
}

std::vector<std::array<double, 3>> load_attack_speed_caps(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs) {
        throw std::runtime_error("Failed to open attack speed cap file: " + filepath);
    }

    json root;
    ifs >> root;
    const json* caps_json = nullptr;
    if (root.is_array()) {
        caps_json = &root;
    } else if (root.is_object() && root.contains("speed_caps")) {
        caps_json = &root.at("speed_caps");
    } else if (root.is_object() && root.contains("caps")) {
        caps_json = &root.at("caps");
    }
    if (caps_json == nullptr || !caps_json->is_array()) {
        if (root.is_object() && root.contains("window_caps")) {
            return {};
        }
        throw std::runtime_error(
            "Attack speed cap file must be an array or contain speed_caps/caps array: "
            + filepath);
    }

    std::vector<std::array<double, 3>> caps;
    caps.reserve(caps_json->size());
    for (const auto& item : *caps_json) {
        double s = std::numeric_limits<double>::quiet_NaN();
        double speed = std::numeric_limits<double>::quiet_NaN();
        double radius = 15.0;
        if (item.is_array() && item.size() >= 2) {
            s = item.at(0).get<double>();
            speed = item.at(1).get<double>();
            if (item.size() >= 3) {
                radius = item.at(2).get<double>();
            }
        } else if (item.is_object()) {
            if (item.contains("s")) {
                s = item.at("s").get<double>();
            } else if (item.contains("s_m")) {
                s = item.at("s_m").get<double>();
            }
            if (item.contains("speed")) {
                speed = item.at("speed").get<double>();
            } else if (item.contains("speed_mps")) {
                speed = item.at("speed_mps").get<double>();
            } else if (item.contains("v")) {
                speed = item.at("v").get<double>();
            }
            if (item.contains("radius")) {
                radius = item.at("radius").get<double>();
            } else if (item.contains("radius_m")) {
                radius = item.at("radius_m").get<double>();
            }
        }

        if (!std::isfinite(s) || !std::isfinite(speed) || !std::isfinite(radius)) {
            throw std::runtime_error("Attack speed cap entry contains non-finite values: " + filepath);
        }
        if (speed <= 0.0 || radius <= 0.0) {
            throw std::runtime_error("Attack speed cap speed/radius must be positive: " + filepath);
        }
        caps.push_back({s, speed, radius});
    }
    return caps;
}

std::vector<std::array<double, 3>> load_attack_speed_windows(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs) {
        throw std::runtime_error("Failed to open attack speed cap file: " + filepath);
    }

    json root;
    ifs >> root;
    if (!root.is_object() || !root.contains("window_caps")) {
        return {};
    }
    const auto& windows_json = root.at("window_caps");
    if (!windows_json.is_array()) {
        throw std::runtime_error("Attack speed window_caps must be an array: " + filepath);
    }

    std::vector<std::array<double, 3>> windows;
    windows.reserve(windows_json.size());
    for (const auto& item : windows_json) {
        double s_start = std::numeric_limits<double>::quiet_NaN();
        double s_end = std::numeric_limits<double>::quiet_NaN();
        double speed = std::numeric_limits<double>::quiet_NaN();
        if (item.is_array() && item.size() >= 3) {
            s_start = item.at(0).get<double>();
            s_end = item.at(1).get<double>();
            speed = item.at(2).get<double>();
        } else if (item.is_object()) {
            if (item.contains("s_start")) {
                s_start = item.at("s_start").get<double>();
            } else if (item.contains("start")) {
                s_start = item.at("start").get<double>();
            } else if (item.contains("start_s")) {
                s_start = item.at("start_s").get<double>();
            }
            if (item.contains("s_end")) {
                s_end = item.at("s_end").get<double>();
            } else if (item.contains("end")) {
                s_end = item.at("end").get<double>();
            } else if (item.contains("end_s")) {
                s_end = item.at("end_s").get<double>();
            }
            if (item.contains("speed")) {
                speed = item.at("speed").get<double>();
            } else if (item.contains("speed_mps")) {
                speed = item.at("speed_mps").get<double>();
            } else if (item.contains("v")) {
                speed = item.at("v").get<double>();
            }
        }

        if (!std::isfinite(s_start) || !std::isfinite(s_end) || !std::isfinite(speed)) {
            throw std::runtime_error("Attack speed window entry contains non-finite values: " + filepath);
        }
        if (speed <= 0.0) {
            throw std::runtime_error("Attack speed window speed must be positive: " + filepath);
        }
        windows.push_back({s_start, s_end, speed});
    }
    return windows;
}

std::string resolve_autocross_csv() {
    const char* env_path = std::getenv("FOPT_AUTOCROSS_CSV");
    if (env_path != nullptr && std::string(env_path).size() > 0) {
        return env_path;
    }

    const std::vector<std::string> candidates = {
        "/workspace/sim/data/ASE_autocross_sim.csv",
        "/workspace/data/ASE_autocross_sim.csv",
        "data/ASE_autocross_sim.csv",
        "sim/data/ASE_autocross_sim.csv",
        "../data/ASE_autocross_sim.csv",
        "../sim/data/ASE_autocross_sim.csv",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "/workspace/sim/data/ASE_autocross_sim.csv";
}

/// ラップタイム結果
struct LapResult {
    double lap_time = 0.0;             // [s]
    double distance_traveled = 0.0;    // [m]
    double max_lateral_error = 0.0;    // [m]
    double avg_lateral_error = 0.0;    // [m]
    double max_speed = 0.0;            // [m/s]
    double avg_speed = 0.0;            // [m/s]
    double rms_heading_error = 0.0;    // [deg]
    double steering_smoothness = 0.0;  // RMS of consecutive steering diffs
    double throttle_smoothness = 0.0;  // RMS of consecutive throttle diffs
    double brake_smoothness = 0.0;     // RMS of consecutive brake diffs
    double steering_saturation_ratio = 0.0; // fraction of samples torque-limited
    double mean_abs_sw_error = 0.0;     // [rad] steering wheel tracking error
    double max_abs_sw_error = 0.0;      // [rad]
    double mean_steering_lag_ratio = 0.0;
    double max_steering_lag_ratio = 0.0;
    double avg_torque_speed_multiplier = 1.0;
    double mean_torque_speed_loss = 0.0;
    double mean_steering_cap_speed_loss = 0.0;
    double mean_steering_reach_speed_loss = 0.0;
    double mean_track_margin_speed_loss = 0.0;
    double mean_corner_throttle_loss = 0.0;
    double mean_chassis_twist_risk = 0.0;
    double chassis_twist_risk_integral = 0.0; // [s], integral of normalized twist risk
    double control_intervention_cost = 0.0;   // control-side rescue cost excluding plant health
    double vehicle_health_penalty = 0.0;      // plant-side penalty, currently normalized chassis twist risk
    double driver_masking_risk = 0.0;         // risk that slow driver intervention is hiding a poor plant
    double driver_intervention_cost = 0.0;    // backward-compatible aggregate = control + vehicle health
    double mean_mpc_rollout_progress_deficit = 0.0; // [m], MPC horizon progress shortfall
    double max_mpc_rollout_progress_deficit = 0.0;  // [m]
    double mean_mpc_predicted_min_margin = 0.0;      // [m]
    double min_mpc_predicted_min_margin = 0.0;       // [m], 0 when no MPC debug was observed
    double mean_mpc_predicted_terminal_abs_n = 0.0;  // [m]
    double mean_mpc_predicted_terminal_abs_heading = 0.0; // [rad]
    double mean_mpc_predicted_terminal_speed = 0.0;  // [m/s]
    double mean_mpc_predicted_signed_progress = 0.0; // [m]
    int last_mpc_selected_plan_index = -1;
    double mean_attack_cost_tracking = 0.0;
    double mean_attack_cost_speed = 0.0;
    double mean_attack_cost_boundary = 0.0;
    double mean_attack_cost_control_smoothness = 0.0;
    double mean_attack_cost_progress = 0.0;
    double mean_attack_cost_primitive_bias = 0.0;
    double mean_attack_cost_overspeed = 0.0;
    double mean_attack_cost_underspeed = 0.0;
    double mean_attack_cost_time_progress = 0.0;
    double mean_attack_rollout_reject_count = 0.0;
    int max_attack_rollout_reject_count = 0;
    double mean_attack_rollout_boundary_reject_count = 0.0;
    double mean_attack_rollout_nonfinite_reject_count = 0.0;
    double mean_attack_best_tracking_cost = 0.0;
    double mean_attack_best_boundary_recovery_cost = 0.0;
    double mean_attack_best_low_speed_pure_pursuit_cost = 0.0;
    double mean_attack_best_speed_scale_065_cost = 0.0;
    double mean_attack_best_speed_scale_080_cost = 0.0;
    double mean_attack_best_speed_scale_095_cost = 0.0;
    double mean_attack_best_speed_scale_105_cost = 0.0;
    double mean_attack_best_speed_scale_095_cost_delta = 0.0;
    double mean_attack_best_speed_scale_095_tracking_delta = 0.0;
    double mean_attack_best_speed_scale_095_speed_delta = 0.0;
    double mean_attack_best_speed_scale_095_boundary_delta = 0.0;
    double mean_attack_best_speed_scale_095_control_smoothness_delta = 0.0;
    double mean_attack_best_speed_scale_095_progress_delta = 0.0;
    double mean_attack_best_speed_scale_095_primitive_bias_delta = 0.0;
    int last_attack_selected_primitive = -1;
    std::string last_attack_selected_primitive_kind = "unknown";
    double last_attack_selected_speed_scale = 0.0;
    double last_attack_selected_target_speed = 0.0;
    double last_attack_selected_ax = 0.0;
    double last_course_v_ref = 0.0;
    double last_course_n_ref = 0.0;
    double last_course_kappa_ref = 0.0;
    double last_course_a_ref = 0.0;
    double mean_course_speed_error = 0.0;
    double mean_abs_course_lateral_error = 0.0;
    double mean_course_target_gap = 0.0;
    double mean_dynamic_cap_gap = 0.0;
    double mean_attack_residual_delta_v_ref = 0.0;
    double mean_attack_residual_delta_steer_ff = 0.0;
    double mean_attack_residual_delta_ax_bias = 0.0;
    double mean_attack_residual_delta_heading_relax = 0.0;
    double attack_residual_safety_gate_ratio = 0.0;
    double mean_attack_effective_heading_error = 0.0;
    double last_attack_residual_delta_v_ref = 0.0;
    double last_attack_residual_delta_steer_ff = 0.0;
    double last_attack_residual_delta_ax_bias = 0.0;
    double last_attack_residual_delta_heading_relax = 0.0;
    bool last_attack_residual_safety_gate_active = false;
    double last_attack_raw_heading_error = 0.0;
    double last_attack_effective_heading_error = 0.0;
    double last_attack_ax_before_residual = 0.0;
    double last_attack_ax_after_residual = 0.0;
    double mean_accel_headroom = 0.0;
    double healthy_underspeed_ratio = 0.0;
    std::string last_longitudinal_mode = "unknown";
    std::string last_track_phase = "unknown";
    int last_attack_segment_id = -1;
    bool last_attack_allowed = false;
    double last_attack_static_speed_cap = 0.0;
    double last_attack_dynamic_speed_cap = 0.0;
    double mean_phase_speed_bias = 1.0;
    double mean_phase_margin_bias = 0.0;
    double mean_phase_progress_bias = 1.0;
    bool last_attack_all_rejected = false;
    double min_steering_actuator_speed_cap = 0.0; // [m/s], 0 when no finite cap was observed
    double min_steering_reach_speed_cap = 0.0;    // [m/s], 0 when no finite cap was observed
    double min_track_margin_speed_cap = 0.0;      // [m/s], 0 when no finite cap was observed
    double min_velocity_profile_steering_cap = 0.0; // [m/s], planner-level steering feasibility cap
    double velocity_profile_steering_cap_ratio = 0.0; // fraction of profile points capped by steering feasibility
    bool velocity_profile_uses_planned_line = false;
    std::string velocity_profile_source = "generated";
    std::string velocity_profile_path = "";
    std::string velocity_profile_fingerprint = "";
    int velocity_profile_point_count = 0;
    double max_abs_phi_twist = 0.0;     // [rad]
    double max_abs_dphi_twist = 0.0;    // [rad/s]
    double max_track_limit_excess = 0.0; // [m] positive when outside track width
    double section_straight_ratio = 0.0;
    double section_entry_ratio = 0.0;
    double section_apex_ratio = 0.0;
    double section_exit_ratio = 0.0;
    double dominant_loss_none_ratio = 0.0;
    double dominant_loss_steering_saturation_ratio = 0.0;
    double dominant_loss_steering_lag_ratio = 0.0;
    double dominant_loss_track_margin_ratio = 0.0;
    double dominant_loss_yaw_mismatch_ratio = 0.0;
    double dominant_loss_front_slip_ratio = 0.0;
    double dominant_loss_chassis_twist_ratio = 0.0;
    std::string dominant_loss = "none";
    bool lap_completed = false;
    std::string status = "success";
    std::string failure_code = "";
    std::string failure_message = "";
    bool degraded_mode = false;
    std::string degraded_reason = "";
    std::string track_source = "builtin";
    std::string track_csv_path = "";
    std::string artifact_defaults = "";  // missing strict groups when --allow-defaults (#2)
    std::string tire_model_type = "tmeasy";
    std::string tire_model_name = "";
    std::string tire_runtime = "tmeasy";
    double failure_time = -1.0;
    // MPCソルバー統計（MPCドライバー使用時のみ有効）
    int    mpc_solve_count = 0;
    double mpc_avg_solve_time_ms = 0.0;
    double mpc_max_solve_time_ms = 0.0;
    int    mpc_fail_count = 0;
    int    actuator_prediction_count = 0;
    double actuator_prediction_bias = 0.0; // [rad], predicted - aligned actual
    double actuator_prediction_mae = 0.0;  // [rad]
    double actuator_prediction_rmse = 0.0; // [rad]
    double actuator_prediction_max_abs_error = 0.0; // [rad]
};

enum class DriverSection : std::size_t {
    Straight = 0,
    Entry = 1,
    Apex = 2,
    Exit = 3,
    Count = 4,
};

enum class DriverLoss : std::size_t {
    None = 0,
    SteeringSaturation = 1,
    SteeringLag = 2,
    TrackMargin = 3,
    YawMismatch = 4,
    FrontSlip = 5,
    ChassisTwist = 6,
    Count = 7,
};

DriverSection classify_driver_section(const driver::PIDDebugInfo& debug) {
    constexpr double kCornerThreshold = 0.008; // [1/m], matches line-planner straight threshold.
    const double now = std::abs(debug.line_curvature_now);
    const double ahead = std::abs(debug.line_curvature_ahead);
    const double behind = std::abs(debug.line_curvature_behind);
    const double max_curvature = std::max({now, ahead, behind});

    if (max_curvature < kCornerThreshold) {
        return DriverSection::Straight;
    }
    if (ahead > now + kCornerThreshold && ahead >= behind) {
        return DriverSection::Entry;
    }
    if (behind > now + kCornerThreshold && behind > ahead) {
        return DriverSection::Exit;
    }
    return DriverSection::Apex;
}

double normalized_speed_cap_loss(double cap, double target_speed_preview) {
    if (!std::isfinite(cap) || target_speed_preview <= 1e-6) {
        return 0.0;
    }
    return std::clamp((target_speed_preview - cap) / target_speed_preview, 0.0, 1.0);
}

DriverLoss classify_dominant_driver_loss(const driver::PIDDebugInfo& debug) {
    std::array<double, static_cast<std::size_t>(DriverLoss::Count)> scores{};
    scores[static_cast<std::size_t>(DriverLoss::SteeringSaturation)] =
        debug.steering_saturation > 0.5 ? 1.0 : 0.0;
    scores[static_cast<std::size_t>(DriverLoss::SteeringLag)] = std::max(
        std::max(
            normalized_speed_cap_loss(debug.steering_actuator_speed_cap, debug.target_speed_preview),
            normalized_speed_cap_loss(debug.steering_reach_speed_cap, debug.target_speed_preview)),
        std::clamp((debug.steering_lag_ratio - 0.08) / 0.25, 0.0, 1.0));
    scores[static_cast<std::size_t>(DriverLoss::TrackMargin)] =
        normalized_speed_cap_loss(debug.track_margin_speed_cap, debug.target_speed_preview);
    scores[static_cast<std::size_t>(DriverLoss::YawMismatch)] =
        std::clamp(debug.stability_yaw_penalty, 0.0, 1.0);
    scores[static_cast<std::size_t>(DriverLoss::FrontSlip)] =
        std::clamp(debug.stability_slip_penalty, 0.0, 1.0);
    scores[static_cast<std::size_t>(DriverLoss::ChassisTwist)] =
        std::clamp(debug.chassis_twist_risk, 0.0, 1.0);

    std::size_t best_index = static_cast<std::size_t>(DriverLoss::None);
    double best_score = 0.0;
    for (std::size_t i = 1; i < scores.size(); ++i) {
        if (scores[i] > best_score) {
            best_score = scores[i];
            best_index = i;
        }
    }

    return best_score > 0.03 ? static_cast<DriverLoss>(best_index) : DriverLoss::None;
}

const char* driver_loss_name(DriverLoss loss) {
    switch (loss) {
        case DriverLoss::SteeringSaturation: return "steering_saturation";
        case DriverLoss::SteeringLag: return "steering_lag";
        case DriverLoss::TrackMargin: return "track_margin";
        case DriverLoss::YawMismatch: return "yaw_mismatch";
        case DriverLoss::FrontSlip: return "front_slip";
        case DriverLoss::ChassisTwist: return "chassis_twist";
        case DriverLoss::None:
        case DriverLoss::Count:
        default:
            return "none";
    }
}

struct DiagnosticFailure : public std::runtime_error {
    std::string code;

    DiagnosticFailure(std::string failure_code, const std::string& message)
        : std::runtime_error(message), code(std::move(failure_code)) {}
};

std::string escape_json_string(const std::string& input) {
    std::ostringstream oss;
    for (unsigned char c : input) {
        switch (c) {
            case '\\': oss << "\\\\"; break;
            case '"': oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default:
                if (c < 0x20) {
                    oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(c) << std::dec << std::setfill(' ');
                } else {
                    oss << static_cast<char>(c);
                }
        }
    }
    return oss.str();
}

void validate_laptime_vehicle_params(const vehicle::VehicleParams& params) {
    auto require_positive = [](double value, const std::string& name, double min_value) {
        if (!std::isfinite(value) || value <= min_value) {
            std::ostringstream oss;
            oss << name << " must be finite and > " << min_value << " (got " << value << ")";
            throw DiagnosticFailure("invalid_vehicle_params", oss.str());
        }
    };

    require_positive(params.mass.m_total, "mass.m_total", 1.0);
    require_positive(params.tire.mu_x_max, "tire.mu_x_max", 0.1);
    require_positive(params.tire.mu_y_max, "tire.mu_y_max", 0.1);
    require_positive(params.tire.radius, "tire.radius", 0.01);
}

/// トラック生成
track::CurvatureBasedTrack create_track(const SimulationConfig& config, LapResult* result = nullptr) {
    // Check if track_name is a CSV file
    if (config.track_name.length() > 4 && 
        config.track_name.substr(config.track_name.length() - 4) == ".csv") {
        if (result != nullptr) {
            result->track_source = "csv";
            result->track_csv_path = config.track_name;
        }
        std::cout << "Loading track from CSV: " << config.track_name << std::endl;
        try {
            auto waypoints = track::WaypointTrackBuilder::load_from_csv(config.track_name, config.track_width, config.track_closed);
            return track::WaypointTrackBuilder(waypoints, config.track_width).build();
        } catch (const std::exception& e) {
            throw DiagnosticFailure(
                "track_csv_load_failed",
                "Failed to load track CSV '" + config.track_name + "': " + e.what());
        }
    }

    if (config.track_name == "circle") {
        return track::create_circle_track(config.track_radius, config.track_width);
    } else if (config.track_name == "circle_cw") {
        return track::create_circle_cw_track(config.track_radius, config.track_width);
    } else if (config.track_name == "figure_eight") {
        return track::create_figure_eight_track(config.track_radius, config.track_width);
    } else if (config.track_name == "skidpad") {
        // FSAE標準: R=9.125m（--radiusで上書き可能）
        double skidpad_r = (config.track_radius != 30.0) ? config.track_radius : 9.125;
        return track::create_skidpad_track(skidpad_r, config.track_width);
    } else if (config.track_name == "straight_corner") {
        return track::create_straight_corner_track(50.0, config.track_radius, config.track_width);
    } else if (config.track_name == "acceleration" || config.track_name == "accel") {
        double length = 75.0;  // FSAEアクセラレーション = 75m
        return track::create_acceleration_track(length, config.track_width);
    } else if (config.track_name == "autocross" || config.track_name == "ASE_autocross") {
        std::string csv_path = resolve_autocross_csv();
        if (result != nullptr) {
            result->track_source = "autocross_csv";
            result->track_csv_path = csv_path;
        }
        try {
            auto waypoints = track::WaypointTrackBuilder::load_from_csv(csv_path, config.track_width, config.track_closed);
            return track::WaypointTrackBuilder(waypoints, config.track_width).build();
        } catch (const std::exception& e) {
            throw DiagnosticFailure(
                "autocross_csv_load_failed",
                "Failed to load autocross CSV '" + csv_path + "': " + e.what());
        }
    } else {
        std::cerr << "Unknown track type: " << config.track_name << ", using circle" << std::endl;
        return track::create_circle_track(config.track_radius, config.track_width);
    }
}

/// PIDゲイン選択
void apply_line_planner_config(driver::PIDGains& gains, const SimulationConfig& config) {
    if (config.line_planner_enabled) {
        gains.line_planner.enabled = true;
    }
    if (config.line_offset_fraction >= 0.0) {
        gains.line_planner.offset_fraction = config.line_offset_fraction;
        gains.line_planner.enabled = true;
    }
    if (config.line_safety_margin >= 0.0) {
        gains.line_planner.safety_margin = config.line_safety_margin;
        gains.line_planner.enabled = true;
    }
    if (config.line_preview_distance >= 0.0) {
        gains.line_planner.preview_distance = config.line_preview_distance;
        gains.line_planner.enabled = true;
    }
    if (config.line_smoothing_distance >= 0.0) {
        gains.line_planner.smoothing_distance = config.line_smoothing_distance;
        gains.line_planner.enabled = true;
    }
    if (config.line_scan_step >= 0.0) {
        gains.line_planner.scan_step = config.line_scan_step;
        gains.line_planner.enabled = true;
    }
    if (config.line_entry_offset_scale >= 0.0) {
        gains.line_planner.entry_offset_scale = config.line_entry_offset_scale;
        gains.line_planner.enabled = true;
    }
    if (config.line_apex_offset_scale >= 0.0) {
        gains.line_planner.apex_offset_scale = config.line_apex_offset_scale;
        gains.line_planner.enabled = true;
    }
    if (config.line_exit_offset_scale >= 0.0) {
        gains.line_planner.exit_offset_scale = config.line_exit_offset_scale;
        gains.line_planner.enabled = true;
    }
    if (config.line_track_geometry_enabled) {
        gains.line_planner.track_geometry = true;
        gains.line_planner.enabled = true;
    }
    if (config.line_track_geometry_disabled) {
        gains.line_planner.track_geometry = false;
    }
    if (config.line_planner_disabled) {
        gains.line_planner.enabled = false;
    }
}

driver::PIDGains select_gains(const SimulationConfig& config) {
    driver::PIDGains gains;

    // プリセットまたはデフォルト
    if (config.gain_preset == "conservative") {
        gains = driver::conservative_gains();
    } else if (config.gain_preset == "aggressive") {
        gains = driver::aggressive_gains();
    } else if (config.gain_preset == "optimized_figure_eight") {
        gains = driver::optimized_figure_eight_gains();
    } else if (config.gain_preset == "optimized_skidpad") {
        gains = driver::optimized_skidpad_gains();
    } else if (config.gain_preset == "optimized_autocross") {
        gains = driver::optimized_autocross_gains();
    } else if (config.gain_preset == "optimized_autocross_fast") {
        gains = driver::optimized_autocross_fast_gains();
    } else {
        // デフォルト: トラック名に基づいて最適なプリセットを自動選択
        // eval backend を比較可能に保つため、fast/full で暗黙に controller を切り替えない。
        // reduced-order backend 向け再調整は --gain-preset optimized_autocross_fast を明示指定する。
        if (is_autocross_like_track(config)) {
            // Autocross/CSVトラックの場合は、最適化済みのAutocrossゲインを適用
            // （FSAE Japanのオートクロスに最適化された値）
            gains = driver::optimized_autocross_gains();
            if (!config.output_json_result) {
                std::cout << "Using optimized Autocross gains by default." << std::endl;
            }
        } else if (config.track_name == "skidpad") {
            gains = driver::optimized_skidpad_gains();
            if (!config.output_json_result) std::cout << "Using optimized Skidpad gains by default." << std::endl;
        } else {
            gains = driver::PIDGains();  // default
        }
    }

    return gains;
}

void apply_pid_gain_overrides(driver::PIDGains& gains, const SimulationConfig& config) {
    if (config.Kp_steer >= 0.0) gains.Kp_steer = config.Kp_steer;
    if (config.Kd_steer >= 0.0) gains.Kd_steer = config.Kd_steer;
    if (config.Ki_steer >= 0.0) gains.Ki_steer = config.Ki_steer;
    if (config.Kh_steer >= 0.0) gains.Kh_steer = config.Kh_steer;
    if (config.Kff_steer >= 0.0) gains.Kff_steer = config.Kff_steer;
    if (config.Kp_speed >= 0.0) gains.Kp_speed = config.Kp_speed;
    if (config.Ki_speed >= 0.0) gains.Ki_speed = config.Ki_speed;
    if (config.max_throttle >= 0.0) gains.max_throttle = config.max_throttle;
    if (config.max_steer >= 0.0) gains.max_steer = config.max_steer;
    if (config.brake_gain_ratio >= 0.0) gains.brake_gain_ratio = config.brake_gain_ratio;
    if (config.corner_throttle_reduction >= 0.0) gains.corner_throttle_reduction = config.corner_throttle_reduction;
    if (config.corner_throttle_torque_reference >= 0.0) {
        gains.corner_throttle_torque_reference = config.corner_throttle_torque_reference;
    }
    if (config.corner_throttle_lateral_utilization_start >= 0.0) {
        gains.corner_throttle_lateral_utilization_start =
            config.corner_throttle_lateral_utilization_start;
    }
    if (config.corner_throttle_lateral_utilization_full >= 0.0) {
        gains.corner_throttle_lateral_utilization_full =
            config.corner_throttle_lateral_utilization_full;
    }
    if (config.corner_throttle_drive_slip_start >= 0.0) {
        gains.corner_throttle_drive_slip_start = config.corner_throttle_drive_slip_start;
    }
    if (config.corner_throttle_drive_slip_full >= 0.0) {
        gains.corner_throttle_drive_slip_full = config.corner_throttle_drive_slip_full;
    }
    if (config.target_speed_preview_distance >= 0.0) {
        gains.target_speed_preview_distance = config.target_speed_preview_distance;
    }
    if (config.steering_preview_distance >= 0.0) {
        gains.steering_preview_distance = config.steering_preview_distance;
    }
    if (config.steering_preview_time >= 0.0) {
        gains.steering_preview_time = config.steering_preview_time;
    }
    if (config.steering_preview_min_distance >= 0.0) {
        gains.steering_preview_min_distance = config.steering_preview_min_distance;
    }
    if (config.steering_preview_max_distance >= 0.0) {
        gains.steering_preview_max_distance = config.steering_preview_max_distance;
    }
    if (config.kpp_steer >= 0.0) {
        gains.Kpp_steer = config.kpp_steer;
    }
    if (config.understeer_gradient >= 0.0) {
        gains.understeer_gradient = config.understeer_gradient;
    }
    if (config.torque_speed_cap_preview_time >= 0.0) {
        gains.torque_speed_cap_preview_time = config.torque_speed_cap_preview_time;
    }
    if (config.torque_speed_cap_preview_distance >= 0.0) {
        gains.torque_speed_cap_preview_distance = config.torque_speed_cap_preview_distance;
    }
    if (config.torque_speed_cap_min_preview_distance >= 0.0) {
        gains.torque_speed_cap_min_preview_distance = config.torque_speed_cap_min_preview_distance;
    }
    if (config.torque_speed_cap_max_preview_distance >= 0.0) {
        gains.torque_speed_cap_max_preview_distance = config.torque_speed_cap_max_preview_distance;
    }
    apply_line_planner_config(gains, config);
}

std::array<double, driver::RLDriver::OBS_DIM> build_rl_teacher_observation(
    const vehicle::State14DOF& st,
    const track::CurvatureBasedTrack& track_model,
    track::FrenetConverter& converter,
    const vehicle::DriverInput& prev_input,
    const vehicle::VehicleParams& params)
{
    std::array<double, driver::RLDriver::OBS_DIM> obs{};
    const double map_x = st.y;
    const double map_y = st.x;
    const double map_theta = M_PI / 2.0 - st.psi;
    auto frenet = converter.global_to_frenet(map_x, map_y, map_theta);
    auto tp = track_model.get_point(frenet.s);

    obs[0] = st.vx;
    obs[1] = st.vy;
    obs[2] = st.r;
    obs[3] = -st.vy * st.r;
    obs[4] = st.vx * st.r;
    obs[5] = st.phi;
    obs[6] = prev_input.steering;
    obs[7] = prev_input.throttle;
    obs[8] = prev_input.brake;

    for (int i = 0; i < 4; ++i) {
        double vw = st.omega[i] * params.tire.radius;
        double vref = std::max(std::abs(st.vx), 0.5);
        obs[9 + i] = (vw - st.vx) / vref;
    }

    double alpha_f = 0.0;
    double alpha_r = 0.0;
    if (std::abs(st.vx) > 0.5) {
        alpha_f = std::atan2(st.vy + params.mass.a * st.r, std::max(st.vx, 0.5));
        alpha_r = std::atan2(st.vy - params.mass.b * st.r, std::max(st.vx, 0.5));
    }
    obs[13] = alpha_f;
    obs[14] = alpha_r;

    obs[15] = frenet.n;
    obs[16] = frenet.d_heading;
    obs[17] = track_model.get_curvature(frenet.s);

    double v_ratio = std::clamp(st.vx / 30.0, 0.0, 1.0);
    double lookahead_distances[5] = {10.0, 20.0, 40.0, 60.0, 80.0};
    for (int i = 0; i < 5; ++i) {
        double d = lookahead_distances[i] * (0.3 + 0.7 * v_ratio);
        double s_ahead = frenet.s + d;
        if (s_ahead > track_model.length()) s_ahead -= track_model.length();
        obs[18 + i] = track_model.get_curvature(s_ahead);
    }

    obs[23] = tp.width_left - frenet.n;
    obs[24] = tp.width_right + frenet.n;
    obs[25] = std::fmod(std::max(frenet.s, 0.0), track_model.length()) / track_model.length();

    double width_distances[4] = {10.0, 20.0, 40.0, 60.0};
    for (int i = 0; i < 4; ++i) {
        double s_ahead = frenet.s + width_distances[i];
        if (s_ahead > track_model.length()) s_ahead -= track_model.length();
        auto tp_ahead = track_model.get_point(s_ahead);
        obs[26 + i] = tp_ahead.width_left + tp_ahead.width_right;
    }

    obs[30] = st.phi_twist;
    obs[31] = st.dphi_twist;
    obs[32] = st.delta_sw;
    obs[33] = st.ddelta_sw;
    obs[34] = params.mass.m_total;
    obs[35] = params.aero.CL_A;
    obs[36] = params.aero.CD_A;
    obs[37] = params.tire.mu_y_max;
    obs[38] = params.tire.mu_x_max;
    obs[39] = params.suspension.k_spring[0];
    obs[40] = params.suspension.k_arb_f;
    obs[41] = params.steering.stiffness;
    obs[42] = params.suspension.K_chassis;
    obs[43] = params.suspension.C_chassis;
    obs[44] = params.suspension.I_twist;
    return obs;
}

void write_rl_teacher_header(std::ofstream& ofs) {
    ofs << "time";
    for (int i = 0; i < driver::RLDriver::OBS_DIM; ++i) {
        ofs << ",obs_" << (i + 1);
    }
    ofs << ",act_steer,act_accel\n";
}

void write_rl_teacher_sample(
    std::ofstream& ofs,
    double t,
    const std::array<double, driver::RLDriver::OBS_DIM>& obs,
    const vehicle::DriverInput& input,
    const vehicle::VehicleParams& params)
{
    const double steer_scale = driver::rl_steering_command_limit(params);
    const double act_steer = std::clamp(input.steering / std::max(steer_scale, 1e-9), -1.0, 1.0);
    const double act_accel = std::clamp(input.throttle - input.brake, -1.0, 1.0);

    ofs << std::setprecision(15) << t;
    for (double v : obs) {
        ofs << "," << v;
    }
    ofs << "," << act_steer << "," << act_accel << "\n";
}

/// ラップタイムシミュレーション実行
LapResult run_laptime_simulation(const SimulationConfig& requested_config) {
    SimulationConfig config = requested_config;

    if (config.eval_mode == "fast" && !config.dt_explicit) {
        config.dt = 0.01;
    }
    // FSAE acceleration はスタンディングスタート（Issue #24）
    if (!config.initial_speed_explicit &&
        (config.track_name == "acceleration" || config.track_name == "accel")) {
        config.initial_speed = 0.0;
    }

    auto fail_result = [](LapResult& result, const std::string& code,
                          const std::string& message, double failure_time) {
        result.status = "failed";
        result.failure_code = code;
        result.failure_message = message;
        result.failure_time = failure_time;
    };

    if (!config.output_json_result) {
        std::cout << "=== Lap Time Simulation ===" << std::endl;
        std::cout << "Track: " << config.track_name << std::endl;
        std::cout << "Driver: " << config.driver_mode << std::endl;
        std::cout << "Eval mode: " << config.eval_mode << std::endl;
        std::cout << "Laps: " << config.laps << std::endl;
        std::cout << "Initial speed: " << config.initial_speed << " m/s" << std::endl;
        if (config.use_custom_gains) {
            std::cout << "Custom Gains: Kp_steer=" << config.Kp_steer << ", Kd_steer=" << config.Kd_steer << std::endl;
        }
        std::cout << "Time step: " << (config.dt * 1000.0) << " ms" << std::endl;
    }

    LapResult result;

    // 1. トラック生成
    std::unique_ptr<track::CurvatureBasedTrack> track;
    try {
        track = std::make_unique<track::CurvatureBasedTrack>(create_track(config, &result));
    } catch (const DiagnosticFailure& e) {
        fail_result(result, e.code, e.what(), 0.0);
        return result;
    }
    track::FrenetConverter frenet_converter(*track);
    if (!config.output_json_result) {
        std::cout << "Track length: " << track->length() << " m" << std::endl;
    }

    // 2. 車両パラメータ読み込み
    vehicle::VehicleParams vehicle_params;
    const bool loading_artifact = !config.params_json_str.empty() || !config.params_file.empty();
    const bool strict_artifact = loading_artifact && !config.allow_defaults;
    try {
        if (!config.params_json_str.empty()) {
            vehicle_params = vehicle::VehicleParams::from_json_string(config.params_json_str, strict_artifact);
        } else if (!config.params_file.empty()) {
            vehicle_params = vehicle::VehicleParams::from_json(config.params_file, strict_artifact);
        }
    } catch (const std::exception& e) {
        throw DiagnosticFailure("strict_artifact_incomplete", e.what());
    }
    if (loading_artifact) {
        const auto missing = vehicle_params.missing_required_groups();
        for (size_t i = 0; i < missing.size(); ++i) {
            result.artifact_defaults += (i ? "," : "") + missing[i];
        }
        if (!result.artifact_defaults.empty()) {
            std::cerr << "[params] using defaults for: " << result.artifact_defaults << std::endl;
        }
    }
    if (config.steering_driver_torque_max >= 0.0) {
        if (config.steering_driver_torque_max <= 0.0) {
            fail_result(result, "invalid_driver_torque_max",
                        "--driver-torque-max must be positive", 0.0);
            return result;
        }
        vehicle_params.steering.T_driver_max = config.steering_driver_torque_max;
    }
    if (config.steering_driver_torque_rate_max >= 0.0) {
        vehicle_params.steering.T_driver_rate_max = config.steering_driver_torque_rate_max;
    }
    validate_laptime_vehicle_params(vehicle_params);
    result.tire_model_type = vehicle_params.tire.model_type;
    result.tire_model_name = vehicle_params.tire.model_name;
    result.tire_runtime = vehicle_params.tire.model_type;
    if (config.eval_mode == "fast") {
        if (vehicle_params.steering.torque_mode) {
            fail_result(
                result,
                "fast_eval_unsupported",
                "fast_eval currently supports steering.torque_mode=false only; steering SAT feedback is not modeled in the surrogate backend.",
                0.0);
            return result;
        }
        result.degraded_mode = true;
        result.degraded_reason =
            "fast_eval uses a reduced-order 6DOF TMeasy surrogate instead of the full 14DOF tire runtime; DRS is not modeled";
    }

    // 3. 速度プロファイル生成
    laptime::VehiclePerformance perf;
    perf.max_lat_accel = config.max_lat_accel;
    perf.enforce_max_lat_accel = config.max_lat_accel_explicit;
    perf.max_lon_accel = config.max_lon_accel;
    perf.max_lon_decel = config.max_lon_decel;
    perf.max_speed = config.max_speed;
    perf.use_physics_model = true;
    perf.mass = vehicle_params.mass.m_total;
    perf.cl_a = vehicle_params.aero.CL_A;
    perf.cd_a = vehicle_params.aero.CD_A;
    perf.rho = vehicle_params.aero.rho > 0.1 ? vehicle_params.aero.rho : 1.225;
    perf.mu_lat = vehicle_params.tire.mu_y_max;
    perf.mu_lon = vehicle_params.tire.mu_x_max;
    perf.friction_margin = config.friction_margin;  // 単一マージン（Issue #26）、旧 safety_factor 0.95 は廃止
    perf.use_load_sensitive_lateral_limit =
        config.eval_mode == "fast" || vehicle_params.tire.model_type == "tmeasy";
    perf.tire_nominal_load = vehicle_params.tire.Fz_nom;
    perf.tire_load_sensitivity = vehicle_params.tire.dmu_dFz;
    perf.front_static_load_fraction = vehicle_params.mass.wheelbase > 1e-6
        ? vehicle_params.mass.b / vehicle_params.mass.wheelbase
        : 0.50;
    perf.front_aero_balance = vehicle_params.aero.aero_balance;
    perf.track_front = vehicle_params.mass.track_f;
    perf.track_rear = vehicle_params.mass.track_r;
    perf.cg_height = vehicle_params.mass.h_cg;
    const double front_spring_roll_stiffness =
        0.5 * (vehicle_params.suspension.k_spring[0] + vehicle_params.suspension.k_spring[1]) *
        std::pow(0.5 * (vehicle_params.suspension.motion_ratio[0] +
                        vehicle_params.suspension.motion_ratio[1]), 2.0) *
        0.5 * vehicle_params.mass.track_f * vehicle_params.mass.track_f +
        vehicle_params.suspension.k_arb_f;
    const double rear_spring_roll_stiffness =
        0.5 * (vehicle_params.suspension.k_spring[2] + vehicle_params.suspension.k_spring[3]) *
        std::pow(0.5 * (vehicle_params.suspension.motion_ratio[2] +
                        vehicle_params.suspension.motion_ratio[3]), 2.0) *
        0.5 * vehicle_params.mass.track_r * vehicle_params.mass.track_r +
        vehicle_params.suspension.k_arb_r;
    perf.front_lateral_transfer_fraction =
        front_spring_roll_stiffness + rear_spring_roll_stiffness > 1e-6
        ? front_spring_roll_stiffness /
            (front_spring_roll_stiffness + rear_spring_roll_stiffness)
        : 0.50;
    perf.aero_nominal_front_ride_height_mm = vehicle_params.aero.nominal_front_ride_height_mm > 0.0
        ? vehicle_params.aero.nominal_front_ride_height_mm
        : 40.0;
    perf.aero_nominal_rear_ride_height_mm = vehicle_params.aero.nominal_rear_ride_height_mm > 0.0
        ? vehicle_params.aero.nominal_rear_ride_height_mm
        : 55.0;
    perf.aero_nominal_roll_rad = vehicle_params.aero.nominal_roll_rad;
    perf.aero_nominal_yaw_rad = vehicle_params.aero.nominal_yaw_rad;

    std::string aero_mode = vehicle_params.aero.map_mode;
    std::transform(aero_mode.begin(), aero_mode.end(), aero_mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    perf.aero_map.params().rho = perf.rho;
    perf.aero_map.set_reference_conditions(vehicle_params.aero.reference_speed, vehicle_params.aero.reference_density);
    perf.aero_map.set_coefficient_defaults(-std::abs(vehicle_params.aero.CL_A),
                                           std::max(vehicle_params.aero.CD_A, 0.01),
                                           vehicle_params.aero.aero_balance);
    perf.use_aero_map = vehicle_params.aero.planner_uses_aero_map;
    if (aero_mode == "direct_load" || aero_mode == "force_map" || aero_mode == "load_map") {
        perf.aero_map.set_direct_load_defaults(
            vehicle_params.aero.Fx_ref_total,
            vehicle_params.aero.Fy_ref_total,
            vehicle_params.aero.Fz_ref_total,
            vehicle_params.aero.Mx_ref_total,
            vehicle_params.aero.My_ref_total,
            vehicle_params.aero.Mz_ref_total
        );
    }
    if (!vehicle_params.aero.aero_map_file.empty()) {
        const bool loaded = perf.aero_map.load_from_json(vehicle_params.aero.aero_map_file);
        if (!loaded) {
            std::cerr << "Warning: failed to load aero map for velocity profile: "
                      << vehicle_params.aero.aero_map_file << std::endl;
        }
    }

    // スキッドパッド: CW→CCW ヨーレート反転を安全にこなすため速度を抑制
    // 理論最大速度 ~13.5 m/s はグリップ限界で交差点過渡応答に余裕なし
    // max_speed=9.5m/s (34km/h) に制限してグリップ使用率 ~50% を確保
    if (config.track_name == "skidpad") {
        perf.max_speed = 9.5;           // [m/s] ≈ 34 km/h
        perf.max_lat_accel = 1.0 * 9.81; // 加速度制限も整合させる
        perf.enforce_max_lat_accel = true;
    }

    if (vehicle_params.steering.torque_mode) {
        const auto torque_gains = driver::optimized_autocross_torque_gains();
        perf.use_steering_actuator_limits = true;
        perf.wheelbase = vehicle_params.mass.wheelbase;
        perf.max_tire_steer_angle = vehicle_params.steering.max_angle;
        perf.max_tire_steer_rate = config.max_tire_steer_rate >= 0.0
            ? config.max_tire_steer_rate
            : torque_gains.max_tire_steer_rate;
        perf.steering_inertia = vehicle_params.steering.I_steer;
        perf.steering_friction_torque = vehicle_params.steering.T_fric;
        perf.steering_driver_torque_max = vehicle_params.steering.T_driver_max;
    }

    driver::PIDGains velocity_profile_gains = select_gains(config);
    if (vehicle_params.steering.torque_mode &&
        config.gain_preset == "default" &&
        is_autocross_like_track(config)) {
        velocity_profile_gains = driver::optimized_autocross_torque_gains();
    }
    apply_pid_gain_overrides(velocity_profile_gains, config);
    if (vehicle_params.steering.torque_mode) {
        perf.steering_response_omega_n = config.steer_inner_omega > 0.0
            ? config.steer_inner_omega
            : velocity_profile_gains.inner_loop_omega_n;
        perf.steering_response_zeta = config.steer_inner_zeta > 0.0
            ? config.steer_inner_zeta
            : velocity_profile_gains.inner_loop_zeta;
        perf.steering_understeer_gradient = velocity_profile_gains.understeer_gradient;
        perf.steering_preview_time = velocity_profile_gains.torque_speed_cap_preview_time;
        perf.steering_preview_distance = velocity_profile_gains.torque_speed_cap_preview_distance;
        perf.steering_min_preview_distance =
            velocity_profile_gains.torque_speed_cap_min_preview_distance;
        perf.steering_max_preview_distance =
            velocity_profile_gains.torque_speed_cap_max_preview_distance;
    }
    if (config.line_profile_curvature && velocity_profile_gains.line_planner.enabled) {
        perf.curvature_override = driver::compute_planned_line_curvature_profile(
            *track,
            velocity_profile_gains.line_planner,
            1.0);
        result.velocity_profile_uses_planned_line = !perf.curvature_override.empty();
    }

    if (config.wind_x != 0.0 || config.wind_y != 0.0) {
        // ponytail: CLI + config only; the 14DOF relative-air-speed/yaw hook is specified in WIRING.md
        std::cerr << "[wind] (" << config.wind_x << ", " << config.wind_y
                  << ") m/s map frame: NOT yet applied — plant hook pending (WIRING.md)" << std::endl;
    }
    if (!config.ggv_file.empty()) {
        try {
            perf.ggv = std::make_shared<const laptime::GGVMap>(laptime::GGVMap::load(config.ggv_file));
        } catch (const std::exception& error) {
            fail_result(result, "ggv_load_failed", error.what(), 0.0);
            return result;
        }
        perf.lon_limits_are_caps = config.max_lon_explicit;
        std::cerr << "[planner] GGV path: " << config.ggv_file << " (" << perf.ggv->source
                  << "), friction_margin=" << perf.friction_margin
                  << (perf.lon_limits_are_caps ? ", --max-lon-* as caps" : "") << std::endl;
    } else {
        std::cerr << "[planner] closed-form path (scalar mu ellipse, max_lon_accel="
                  << perf.max_lon_accel << ", max_lon_decel=" << perf.max_lon_decel
                  << ", friction_margin=" << perf.friction_margin
                  << "); pass --ggv <file> for the 14DOF-derived GGV" << std::endl;
    }

    std::vector<laptime::VelocityPoint> velocity_profile;
    if (!config.velocity_profile_input_file.empty()) {
        try {
            const auto artifact = laptime::load_velocity_profile_artifact(
                config.velocity_profile_input_file, config.track_name, *track);
            velocity_profile = artifact.points;
            result.velocity_profile_source = "file";
            result.velocity_profile_path = config.velocity_profile_input_file;
            result.velocity_profile_fingerprint = artifact.profile_fingerprint;
        } catch (const std::exception& error) {
            fail_result(result, "velocity_profile_load_failed", error.what(), 0.0);
            return result;
        }
    } else {
        laptime::VelocityProfileGenerator vel_gen(perf);
        velocity_profile = vel_gen.generate(*track, 1.0, config.initial_speed);
        if (velocity_profile.size() > 1 && velocity_profile.front().v < 1e-3) {
            // 静止発進: v(0)=0 は位置基準の目標速度として特異（追従器が発進しない）。
            // 先頭点だけ 1 サンプル先の到達可能速度を目標上限にする（プラント初速は 0 のまま）。
            velocity_profile.front().v = velocity_profile[1].v;
        }
        result.velocity_profile_fingerprint =
            laptime::velocity_profile_fingerprint(velocity_profile);
        if (!config.velocity_profile_output_file.empty()) {
            try {
                const auto artifact = laptime::make_velocity_profile_artifact(
                    velocity_profile, config.track_name, *track);
                laptime::save_velocity_profile_artifact(
                    artifact, config.velocity_profile_output_file);
                result.velocity_profile_path = config.velocity_profile_output_file;
            } catch (const std::exception& error) {
                fail_result(result, "velocity_profile_save_failed", error.what(), 0.0);
                return result;
            }
        }
    }
    result.velocity_profile_point_count = static_cast<int>(velocity_profile.size());

    if (perf.use_steering_actuator_limits && !velocity_profile.empty()) {
        double min_profile_steering_cap = std::numeric_limits<double>::infinity();
        int finite_cap_count = 0;
        int active_cap_count = 0;
        for (const auto& point : velocity_profile) {
            if (std::isfinite(point.steering_speed_cap)) {
                finite_cap_count++;
                min_profile_steering_cap =
                    std::min(min_profile_steering_cap, point.steering_speed_cap);
                if (point.steering_speed_cap < perf.max_speed - 1e-6) {
                    active_cap_count++;
                }
            }
        }
        if (std::isfinite(min_profile_steering_cap)) {
            result.min_velocity_profile_steering_cap = min_profile_steering_cap;
        }
        if (finite_cap_count > 0) {
            result.velocity_profile_steering_cap_ratio =
                static_cast<double>(active_cap_count) / static_cast<double>(velocity_profile.size());
        }
    }

    // 速度プロファイルからの推定ラップタイムでシミュレーション時間を自動設定
    // （ユーザーが --time を指定していない場合のみ; config は const なのでローカル変数に保持）
    double sim_time = config.simulation_time;
    if (!config.simulation_time_explicit && !velocity_profile.empty()) {
        // 平均速度からラップタイムを推定
        double vp_avg_speed = 0.0;
        for (const auto& p : velocity_profile) vp_avg_speed += p.v;
        vp_avg_speed /= static_cast<double>(velocity_profile.size());
        if (vp_avg_speed > 1.0) {
            double estimated_laptime = (track->length() / vp_avg_speed) * std::max(1, config.laps);
            if (vehicle_params.steering.torque_mode) {
                const double torque_speed_scale = config.torque_speed_scale >= 0.0
                    ? config.torque_speed_scale
                    : driver::PIDGains().torque_target_speed_scale;
                estimated_laptime /= std::max(0.25, torque_speed_scale);
            }
            // 2倍の余裕を持たせる（PIDの遅れ・コーナリング速度低下を考慮）
            sim_time = std::max(sim_time, estimated_laptime * 2.0);
            if (!config.output_json_result) {
                std::cout << "Auto simulation time: " << sim_time << " s"
                          << " (est. lap=" << estimated_laptime << " s)" << std::endl;
            }
        }
    }

    // 4. ドライバー初期化
    if (!config.output_json_result) std::cout << "DEBUG: Initializing driver..." << std::endl;
    const bool use_rl_driver = (config.driver_mode == "rl");
    const bool use_mpc_driver = (config.driver_mode == "mpc");
    const bool use_mpc_v2_driver = (config.driver_mode == "mpc_v2");
    const bool use_attack_driver = (config.driver_mode == "attack");
    const bool use_lpv_lqr_driver = (config.driver_mode == "lpv_lqr");
    const bool use_predictive_driver =
        use_lpv_lqr_driver || use_mpc_driver || use_mpc_v2_driver || use_attack_driver;
    const bool use_pid_driver = (config.driver_mode == "pid");
    std::unique_ptr<driver::PIDPathFollowingDriver> pid_driver;
    std::unique_ptr<driver::MPCPathFollowingDriver> mpc_driver;
    std::unique_ptr<driver::MPCPathFollowingDriverV2> mpc_v2_driver;
    std::unique_ptr<driver::AttackPathFollowingDriver> attack_driver;
    std::unique_ptr<driver::PreviewLPVLQRDriver> lpv_lqr_driver;
    std::unique_ptr<driver::RLDriver> rl_driver;

    if (use_rl_driver) {
        if (config.rl_actor_dir.empty()) {
            throw std::runtime_error("driver=rl requires --rl-actor-dir");
        }
        rl_driver = std::make_unique<driver::RLDriver>();
        rl_driver->set_vehicle_params(vehicle_params);
        rl_driver->set_steering_torque_rate_limit_override(config.rl_steer_torque_rate_limit);
        rl_driver->load(config.rl_actor_dir);
    }
    if (use_lpv_lqr_driver) {
        driver::LPVLQRConfig lqr_config;
        lqr_config.max_lon_accel = config.max_lon_accel;
        lqr_config.max_lon_decel = config.max_lon_decel;
        lqr_config.max_model_speed = config.max_speed;
        lqr_config.control_dt = 0.025;
        lqr_config.use_actuator_augmented_lqr = (config.lpv_model == "steering6");
        lqr_config.actuator_q[0] *= config.lpv_lateral_weight_scale;
        lqr_config.actuator_q[1] *= config.lpv_heading_weight_scale;
        if (config.steer_inner_omega > 0.0) {
            lqr_config.steering_inner_loop_omega_n = config.steer_inner_omega;
        }
        if (config.steer_inner_zeta > 0.0) {
            lqr_config.steering_inner_loop_zeta = config.steer_inner_zeta;
        }
        lpv_lqr_driver = std::make_unique<driver::PreviewLPVLQRDriver>(
            *track,
            velocity_profile,
            lqr_config,
            vehicle_params);
    } else if (use_predictive_driver) {
        control::MPCConfig mpc_config;
        if (config.mpc_horizon > 0) {
            mpc_config.N = config.mpc_horizon;
        } else if (is_autocross_like_track(config)) {
            mpc_config.N = use_attack_driver ? 40 : 60;
        }
        if (config.mpc_dt > 0.0) {
            mpc_config.dt = config.mpc_dt;
        }
        mpc_config.delta_max = std::min(mpc_config.delta_max, vehicle_params.steering.max_angle);
        mpc_config.vx_max = config.max_speed;
        mpc_config.ax_max = config.max_lon_accel;
        mpc_config.ax_min = -config.max_lon_decel;
        if (vehicle_params.steering.torque_mode) {
            const auto torque_gains = driver::optimized_autocross_torque_gains();
            mpc_config.steering_inner_loop_omega_n = config.steer_inner_omega > 0.0
                ? config.steer_inner_omega
                : torque_gains.inner_loop_omega_n;
            mpc_config.steering_inner_loop_zeta = config.steer_inner_zeta > 0.0
                ? config.steer_inner_zeta
                : torque_gains.inner_loop_zeta;
        }
        if (use_attack_driver) {
            mpc_config.use_track_margin_speed_cap = true;
            mpc_config.track_margin_speed_cap = 9.0;
            mpc_config.track_margin_speed_release = 1.20;
            mpc_config.hard_boundary_margin = 0.35;
            mpc_config.soft_boundary_margin = 0.90;
            mpc_config.hard_boundary_weight = 1500.0;
            mpc_config.soft_boundary_weight = 140.0;
            mpc_config.outward_rate_weight = 30.0;
            mpc_config.terminal_progress_weight = 1.50;
            mpc_config.progress_target_fraction = 0.92;
            mpc_config.w_y = 7.0;
            mpc_config.w_psi = 2.5;
            mpc_config.w_vx = 0.25;
            mpc_config.w_delta_rate = 0.06;
            mpc_config.w_ax_rate = 0.0005;
            if (!config.driver_capability_profile_file.empty()) {
                const auto profile = driver::load_driver_capability_profile(
                    config.driver_capability_profile_file);
                driver::apply_attack_overrides(profile.attack_overrides, mpc_config);
            }
            // 旧: 質量則 pow(282/m, 0.675) を lateral_safety に追加で掛けていた → 廃止（Issue #26, 係数 1.0）
            if (config.attack_speed_lateral_safety > 0.0) {
                mpc_config.attack_speed_lateral_safety = config.attack_speed_lateral_safety;
            }
            if (config.attack_speed_braking_safety > 0.0) {
                mpc_config.attack_speed_braking_safety = config.attack_speed_braking_safety;
            }
            if (config.attack_speed_accel_limit > 0.0) {
                mpc_config.attack_speed_accel_limit = config.attack_speed_accel_limit;
            }
            if (config.attack_rollout_understeer_gradient >= 0.0) {
                mpc_config.attack_rollout_understeer_gradient =
                    config.attack_rollout_understeer_gradient;
            }
            if (config.attack_floor_straight > 0.0) {
                mpc_config.attack_floor_straight = config.attack_floor_straight;
            }
            if (config.attack_floor_exit > 0.0) {
                mpc_config.attack_floor_exit = config.attack_floor_exit;
            }
            if (config.attack_floor_complex > 0.0) {
                mpc_config.attack_floor_complex = config.attack_floor_complex;
            }
            if (config.attack_floor_turn_in > 0.0) {
                mpc_config.attack_floor_turn_in = config.attack_floor_turn_in;
            }
            if (config.attack_floor_apex > 0.0) {
                mpc_config.attack_floor_apex = config.attack_floor_apex;
            }
            if (config.attack_floor_brake > 0.0) {
                mpc_config.attack_floor_brake = config.attack_floor_brake;
            }
            if (config.attack_braking_corridor_decel_scale >= 0.0) {
                mpc_config.attack_braking_corridor_decel_scale =
                    config.attack_braking_corridor_decel_scale;
            }
            if (config.attack_braking_corridor_band_scale >= 0.0) {
                mpc_config.attack_braking_corridor_band_scale =
                    config.attack_braking_corridor_band_scale;
            }
            if (config.attack_braking_corridor_mild_accel_scale >= 0.0) {
                mpc_config.attack_braking_corridor_mild_accel_scale =
                    config.attack_braking_corridor_mild_accel_scale;
            }
            if (config.attack_braking_corridor_overspeed_brake_gain >= 0.0) {
                mpc_config.attack_braking_corridor_overspeed_brake_gain =
                    config.attack_braking_corridor_overspeed_brake_gain;
            }
            mpc_config.attack_braking_corridor_guard_only =
                config.attack_braking_corridor_guard_only;
            if (config.attack_w_y >= 0.0) {
                mpc_config.w_y = config.attack_w_y;
            }
            if (config.attack_w_psi >= 0.0) {
                mpc_config.w_psi = config.attack_w_psi;
            }
            if (config.attack_w_vx >= 0.0) {
                mpc_config.w_vx = config.attack_w_vx;
            }
            if (config.attack_w_delta >= 0.0) {
                mpc_config.w_delta = config.attack_w_delta;
            }
            if (config.attack_w_ax >= 0.0) {
                mpc_config.w_ax = config.attack_w_ax;
            }
            if (config.attack_w_delta_rate >= 0.0) {
                mpc_config.w_delta_rate = config.attack_w_delta_rate;
            }
            if (config.attack_w_ax_rate >= 0.0) {
                mpc_config.w_ax_rate = config.attack_w_ax_rate;
            }
            if (config.attack_terminal_progress_weight >= 0.0) {
                mpc_config.terminal_progress_weight = config.attack_terminal_progress_weight;
            }
            if (config.attack_progress_target_fraction > 0.0) {
                mpc_config.progress_target_fraction = config.attack_progress_target_fraction;
            }
            if (config.attack_soft_boundary_weight >= 0.0) {
                mpc_config.soft_boundary_weight = config.attack_soft_boundary_weight;
            }
            if (config.attack_outward_rate_weight >= 0.0) {
                mpc_config.outward_rate_weight = config.attack_outward_rate_weight;
            }
            if (!config.attack_speed_cap_file.empty()) {
                mpc_config.attack_speed_caps =
                    load_attack_speed_caps(config.attack_speed_cap_file);
                mpc_config.attack_speed_windows =
                    load_attack_speed_windows(config.attack_speed_cap_file);
            }
            const auto attack_line_planner = velocity_profile_gains.line_planner;
            attack_driver = std::make_unique<driver::AttackPathFollowingDriver>(
                *track,
                velocity_profile,
                mpc_config,
                vehicle_params,
                driver::TrackPhaseProfile(),
                attack_line_planner);
            if (!config.attack_residual_profile_file.empty()) {
                attack_driver->set_residual_profile(
                    driver::load_attack_residual_profile(
                        config.attack_residual_profile_file,
                        track->length(),
                        track->is_closed_loop()));
            }
        } else if (use_mpc_v2_driver) {
            mpc_v2_driver = std::make_unique<driver::MPCPathFollowingDriverV2>(
                *track,
                velocity_profile,
                mpc_config,
                vehicle_params);
        } else {
            mpc_driver = std::make_unique<driver::MPCPathFollowingDriver>(
                *track,
                velocity_profile,
                mpc_config,
                vehicle_params);
        }
    }
    if (use_pid_driver || !config.rl_teacher_output_file.empty()) {
        auto gains = select_gains(config);

        // 車両パラメータのトルクモード設定をPIDゲインに反映
        // A-1: PIDPathFollowingDriver コンストラクタ内の sync_steering_contract() が
        //      vehicle_params.steering から Kp/Kd/Ki を自動正規化するため、
        //      ここでは torque_mode フラグと速度スケール系のみ設定する。
        if (vehicle_params.steering.torque_mode) {
            if (config.gain_preset == "default" && is_autocross_like_track(config)) {
                gains = driver::optimized_autocross_torque_gains();
            }
            // CLIで一部だけ上書きする場合も、トルク用プリセットをベースに差分だけ重ねる。
            apply_pid_gain_overrides(gains, config);

            // torque_mode / R_steer / max_steer_torque / Kp_inner / Kd_inner / Ki_inner は
            // PIDPathFollowingDriver(vehicle_params) コンストラクタの sync_steering_contract() が設定する。
            // inner_loop_auto_tune はデフォルト true なので vehicle_params.steering.I_steer から自動計算される。
            if (gains.target_speed_preview_distance <= 1e-6) {
                gains.target_speed_preview_distance = 4.0;
            }
            if (config.max_throttle < 0.0) {
                gains.max_throttle = std::min(gains.max_throttle, 0.50);
            }
            if (config.brake_gain_ratio < 0.0) {
                gains.brake_gain_ratio = std::max(gains.brake_gain_ratio, 0.80);
            }
            if (config.corner_throttle_torque_reference >= 0.0) {
                gains.corner_throttle_torque_reference = config.corner_throttle_torque_reference;
            }
            if (config.torque_speed_scale >= 0.0) {
                gains.torque_target_speed_scale = config.torque_speed_scale;
            }
            if (config.torque_speed_cap_preview_distance >= 0.0) {
                gains.torque_speed_cap_preview_distance = config.torque_speed_cap_preview_distance;
            }
            if (config.target_speed_preview_distance >= 0.0) {
                gains.target_speed_preview_distance = config.target_speed_preview_distance;
            }
            if (config.torque_preview_distance >= 0.0) {
                gains.target_speed_preview_distance = config.torque_preview_distance;
            }
            if (config.torque_outer_loop_scale >= 0.0) {
                gains.torque_outer_loop_scale = config.torque_outer_loop_scale;
            }
            if (config.steering_preview_distance >= 0.0) {
                gains.steering_preview_distance = config.steering_preview_distance;
            }
            if (config.steering_preview_time >= 0.0) {
                gains.steering_preview_time = config.steering_preview_time;
            }
            if (config.steering_preview_min_distance >= 0.0) {
                gains.steering_preview_min_distance = config.steering_preview_min_distance;
            }
            if (config.steering_preview_max_distance >= 0.0) {
                gains.steering_preview_max_distance = config.steering_preview_max_distance;
            }
            if (config.kpp_steer >= 0.0) {
                gains.Kpp_steer = config.kpp_steer;
            }
            if (config.max_tire_steer_rate >= 0.0) {
                gains.max_tire_steer_rate = config.max_tire_steer_rate;
            }
            if (config.torque_steer_rate_reference_torque >= 0.0) {
                gains.torque_steer_rate_reference_torque = config.torque_steer_rate_reference_torque;
            }
            if (config.torque_steer_rate_reference_inertia >= 0.0) {
                gains.torque_steer_rate_reference_inertia = config.torque_steer_rate_reference_inertia;
            }
            if (config.torque_steer_rate_authority_exponent >= 0.0) {
                gains.torque_steer_rate_authority_exponent = config.torque_steer_rate_authority_exponent;
            }
            if (config.torque_steer_rate_min_scale >= 0.0) {
                gains.torque_steer_rate_min_scale = config.torque_steer_rate_min_scale;
            }
            if (config.torque_steer_rate_max_scale >= 0.0) {
                gains.torque_steer_rate_max_scale = config.torque_steer_rate_max_scale;
            }
            if (config.torque_lag_lead_gain >= 0.0) {
                gains.torque_lag_lead_gain = config.torque_lag_lead_gain;
            }
            if (config.torque_lag_lead_time >= 0.0) {
                gains.torque_lag_lead_time = config.torque_lag_lead_time;
            }
            if (config.torque_lag_lead_max_tire_angle >= 0.0) {
                gains.torque_lag_lead_max_tire_angle = config.torque_lag_lead_max_tire_angle;
            }
            if (config.torque_speed_recovery_gain >= 0.0) {
                gains.torque_speed_recovery_gain = config.torque_speed_recovery_gain;
            }
            if (config.torque_speed_recovery_curvature >= 0.0) {
                gains.torque_speed_recovery_curvature = config.torque_speed_recovery_curvature;
            }
            if (config.torque_speed_recovery_lag_ratio >= 0.0) {
                gains.torque_speed_recovery_lag_ratio = config.torque_speed_recovery_lag_ratio;
            }
            if (config.torque_speed_recovery_lateral_error >= 0.0) {
                gains.torque_speed_recovery_lateral_error = config.torque_speed_recovery_lateral_error;
            }
            if (config.torque_speed_recovery_heading_error >= 0.0) {
                gains.torque_speed_recovery_heading_error = config.torque_speed_recovery_heading_error;
            }
            if (config.torque_speed_recovery_preview_gap >= 0.0) {
                gains.torque_speed_recovery_preview_gap = config.torque_speed_recovery_preview_gap;
            }
            if (config.torque_speed_recovery_reach_time >= 0.0) {
                gains.torque_speed_recovery_reach_time = config.torque_speed_recovery_reach_time;
            }
            if (config.torque_reach_speed_cap >= 0.0) {
                gains.torque_reach_speed_cap = config.torque_reach_speed_cap;
            }
            if (config.torque_reach_time_start >= 0.0) {
                gains.torque_reach_time_start = config.torque_reach_time_start;
            }
            if (config.torque_reach_time_full >= 0.0) {
                gains.torque_reach_time_full = config.torque_reach_time_full;
            }
            if (config.torque_reach_outward_rate_start >= 0.0) {
                gains.torque_reach_outward_rate_start = config.torque_reach_outward_rate_start;
            }
            if (config.torque_reach_outward_rate_full >= 0.0) {
                gains.torque_reach_outward_rate_full = config.torque_reach_outward_rate_full;
            }
            if (config.torque_reach_min_preview_gap >= 0.0) {
                gains.torque_reach_min_preview_gap = config.torque_reach_min_preview_gap;
            }
            if (config.track_margin_speed_cap >= 0.0) {
                gains.track_margin_speed_cap = config.track_margin_speed_cap;
            }
            if (config.track_margin_speed_release >= 0.0) {
                gains.track_margin_speed_release = config.track_margin_speed_release;
            }
            if (config.track_margin_preview_time >= 0.0) {
                gains.track_margin_preview_time = config.track_margin_preview_time;
            }
            if (config.steer_inner_ki >= 0.0) {
                // auto_tune による Kp/Kd 正規化は維持し、Ki だけデバッグ用に上書きする。
                gains.inner_loop_ki_override = config.steer_inner_ki;
            }
            if (config.steer_inner_omega > 0.0) {
                gains.inner_loop_omega_n = config.steer_inner_omega;
            }
            if (config.steer_inner_zeta > 0.0) {
                gains.inner_loop_zeta = config.steer_inner_zeta;
            }
            if (config.torque_stability_vy_threshold >= 0.0) {
                gains.torque_stability_vy_threshold = config.torque_stability_vy_threshold;
            }
            if (config.torque_stability_vy_gain >= 0.0) {
                gains.torque_stability_vy_gain = config.torque_stability_vy_gain;
            }
            if (config.torque_stability_yaw_threshold >= 0.0) {
                gains.torque_stability_yaw_threshold = config.torque_stability_yaw_threshold;
            }
            if (config.torque_stability_yaw_gain >= 0.0) {
                gains.torque_stability_yaw_gain = config.torque_stability_yaw_gain;
            }
            if (config.torque_stability_slip_threshold >= 0.0) {
                gains.torque_stability_slip_threshold = config.torque_stability_slip_threshold;
            }
            if (config.torque_stability_slip_gain >= 0.0) {
                gains.torque_stability_slip_gain = config.torque_stability_slip_gain;
            }
            if (config.distance_cg_front >= 0.0) {
                gains.distance_cg_front = config.distance_cg_front;
            }
            if (!config.output_json_result) {
                std::cout << "Torque steering mode: ON (auto_tune=" << gains.inner_loop_auto_tune
                          << ", omega_n=" << gains.inner_loop_omega_n
                          << " rad/s, I_steer=" << vehicle_params.steering.I_steer
                          << " kg*m2, speed_scale=" << gains.torque_target_speed_scale
                          << ", preview=" << gains.target_speed_preview_distance
                          << " m, lag_lead_gain=" << gains.torque_lag_lead_gain
                          << ", speed_recovery=" << gains.torque_speed_recovery_gain
                          << ", line_planner=" << (gains.line_planner.enabled ? "ON" : "OFF")
                          << ")" << std::endl;
            }
        } else {
            apply_pid_gain_overrides(gains, config);
        }

        pid_driver = std::make_unique<driver::PIDPathFollowingDriver>(*track, velocity_profile, gains, vehicle_params);
    }
    if (!config.output_json_result) std::cout << "DEBUG: Driver initialized." << std::endl;

    // 5. 車両初期化
    if (!config.output_json_result) std::cout << "DEBUG: Initializing vehicle ODE..." << std::endl;
    std::unique_ptr<vehicle::LapVehicle> vehicle;
    try {
        if (config.eval_mode == "fast") {
            vehicle = std::make_unique<vehicle::FastLapVehicle>(vehicle_params);
        } else {
            vehicle = std::make_unique<vehicle::FullLapVehicle>(vehicle_params);
        }
        result.tire_runtime = vehicle->active_tire_runtime_description();
    } catch (const std::exception& e) {
        fail_result(result, "tire_model_load_failed", e.what(), 0.0);
        return result;
    }
    vehicle->reset();
    vehicle->set_initial_velocity(config.initial_speed);
    if (!config.output_json_result) std::cout << "DEBUG: Vehicle ODE initialized." << std::endl;

    auto start_point = track->get_point(0.0);
    // Keep all backends in the same external 14DOF state convention:
    // vehicle X=map Y, vehicle Y=map X, yaw positive clockwise.
    vehicle->set_map_pose(start_point.x, start_point.y, start_point.theta);

    // 6. 出力ファイル準備
    std::string output_filename = config.output_file.empty() ? ("laptime_sim_" + config.track_name + ".csv") : config.output_file;
    std::ofstream ofs;
    if (!config.output_json_result || !config.output_file.empty()) {
        ofs.open(output_filename);
        if (ofs.is_open()) {
            ofs << "time,x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,s,n,d_heading,kappa,steering,throttle,brake,gear,speed,target_speed,speed_error,lateral_error,heading_error_deg,omega_FL,omega_FR,omega_RL,omega_RR,drs_active,target_speed_base,target_speed_preview,target_speed_final,steering_preview_distance_used,target_lateral_offset,line_curvature_now,line_curvature_ahead,line_curvature_behind,delta_target,target_sw,delta_sw,ddelta_sw,required_delta_preview,actual_delta_tire,predicted_delta_tire,preview_delta_gap,raw_delta_target,inner_target_delta_tire,actual_delta_tire_rate_pre,lpv_prediction_error,predicted_delta_tire_rate,steering_prediction_horizon,steering_prediction_target_time,steering_prediction_valid,steering_prediction_error_valid,lpv_outer_sequence,required_reference_s,required_reference_time,model_yaw_rate_map,actual_yaw_rate_map,yaw_rate_error_map,sw_error,torque_lag_lead_sw,T_driver_raw,T_driver_clamped,torque_headroom,T_SAT_total,T_SAT_reflected,T_SAT_feedforward,time_to_reach_delta,steering_saturation,torque_speed_multiplier,torque_speed_recovery_ratio,steering_lag_ratio,chassis_twist_risk,effective_max_tire_steer_rate,steering_actuator_speed_cap,steering_reach_speed_cap,track_margin_speed_cap,track_margin_effective_margin,track_margin_outward_rate,mpc_rollout_progress,mpc_rollout_progress_target,mpc_rollout_progress_deficit,mpc_rollout_cost,mpc_selected_plan_index,mpc_predicted_terminal_n,mpc_predicted_terminal_heading,mpc_predicted_min_margin,mpc_predicted_terminal_speed,mpc_predicted_signed_progress,attack_selected_primitive,attack_selected_primitive_kind,attack_selected_speed_scale,attack_selected_target_speed,attack_selected_ax,course_v_ref,course_n_ref,course_kappa_ref,course_speed_error,course_lateral_error,course_a_ref,course_target_gap,dynamic_cap_gap,braking_corridor_active,braking_corridor_v_lower,braking_corridor_v_upper,attack_residual_delta_v_ref,attack_residual_delta_steer_ff,attack_residual_delta_ax_bias,attack_residual_delta_heading_relax,attack_residual_safety_gate_active,attack_raw_heading_error,attack_effective_heading_error,attack_ax_before_residual,attack_ax_after_residual,longitudinal_mode,healthy_underspeed,accel_headroom,track_phase,phase_speed_bias,phase_margin_bias,phase_progress_bias,attack_segment_id,attack_allowed,attack_static_speed_cap,attack_dynamic_speed_cap,attack_dynamic_base_speed,attack_curvature_speed_cap,attack_steering_speed_cap,attack_margin_speed_cap,attack_heading_speed_cap,attack_outward_speed_cap,attack_limiting_cap_reason,attack_rollout_candidate_count,attack_rollout_reject_count,attack_rollout_boundary_reject_count,attack_rollout_nonfinite_reject_count,attack_all_rejected,attack_cost_tracking,attack_cost_speed,attack_cost_boundary,attack_cost_control_smoothness,attack_cost_progress,attack_cost_primitive_bias,attack_cost_overspeed,attack_cost_underspeed,attack_cost_time_progress,attack_best_tracking_cost,attack_best_boundary_recovery_cost,attack_best_low_speed_pure_pursuit_cost,attack_best_speed_scale_065_cost,attack_best_speed_scale_080_cost,attack_best_speed_scale_095_cost,attack_best_speed_scale_105_cost,attack_best_speed_scale_095_cost_delta,attack_best_speed_scale_095_tracking_delta,attack_best_speed_scale_095_speed_delta,attack_best_speed_scale_095_boundary_delta,attack_best_speed_scale_095_control_smoothness_delta,attack_best_speed_scale_095_progress_delta,attack_best_speed_scale_095_primitive_bias_delta,corner_throttle_steer_ratio,corner_throttle_lateral_utilization,corner_throttle_lateral_ratio,corner_throttle_drive_slip,corner_throttle_drive_slip_ratio,corner_throttle_limit_ratio,corner_throttle_scale,phi_twist,dphi_twist,stability_vy_penalty,stability_yaw_penalty,stability_slip_penalty,stability_penalty,estimated_front_slip\n";
        }
    }
    std::ofstream teacher_ofs;
    track::FrenetConverter teacher_converter(*track);
    vehicle::DriverInput teacher_prev_input;
    double next_teacher_log_time = 0.0;
    constexpr double kTeacherControlPeriod = 0.020;
    if (!config.rl_teacher_output_file.empty()) {
        teacher_ofs.open(config.rl_teacher_output_file);
        if (!teacher_ofs.is_open()) {
            throw std::runtime_error("Failed to open RL teacher output: " + config.rl_teacher_output_file);
        }
        write_rl_teacher_header(teacher_ofs);
    }

    // 7. シミュレーションループ
    double total_lateral_error = 0.0;
    double total_speed = 0.0;
    int sample_count = 0;
    double total_heading_sq = 0.0;
    double total_steer_diff_sq = 0.0;
    double total_thr_diff_sq = 0.0;
    double total_brake_diff_sq = 0.0;
    double total_sw_error_abs = 0.0;
    double total_steering_lag_ratio = 0.0;
    double total_torque_speed_multiplier = 0.0;
    double total_torque_speed_loss = 0.0;
    double total_steering_cap_speed_loss = 0.0;
    double total_steering_reach_speed_loss = 0.0;
    double total_track_margin_speed_loss = 0.0;
    double total_corner_throttle_loss = 0.0;
    double total_chassis_twist_risk = 0.0;
    double total_mpc_rollout_progress_deficit = 0.0;
    double total_mpc_predicted_min_margin = 0.0;
    double total_mpc_predicted_terminal_abs_n = 0.0;
    double total_mpc_predicted_terminal_abs_heading = 0.0;
    double total_mpc_predicted_terminal_speed = 0.0;
    double total_mpc_predicted_signed_progress = 0.0;
    int mpc_progress_metric_count = 0;
    double total_attack_cost_tracking = 0.0;
    double total_attack_cost_speed = 0.0;
    double total_attack_cost_boundary = 0.0;
    double total_attack_cost_control_smoothness = 0.0;
    double total_attack_cost_progress = 0.0;
    double total_attack_cost_primitive_bias = 0.0;
    double total_attack_cost_overspeed = 0.0;
    double total_attack_cost_underspeed = 0.0;
    double total_attack_cost_time_progress = 0.0;
    double total_attack_rollout_reject_count = 0.0;
    double total_attack_rollout_boundary_reject_count = 0.0;
    double total_attack_rollout_nonfinite_reject_count = 0.0;
    double total_attack_best_tracking_cost = 0.0;
    double total_attack_best_boundary_recovery_cost = 0.0;
    double total_attack_best_low_speed_pure_pursuit_cost = 0.0;
    double total_attack_best_speed_scale_065_cost = 0.0;
    double total_attack_best_speed_scale_080_cost = 0.0;
    double total_attack_best_speed_scale_095_cost = 0.0;
    double total_attack_best_speed_scale_105_cost = 0.0;
    double total_attack_best_speed_scale_095_cost_delta = 0.0;
    double total_attack_best_speed_scale_095_tracking_delta = 0.0;
    double total_attack_best_speed_scale_095_speed_delta = 0.0;
    double total_attack_best_speed_scale_095_boundary_delta = 0.0;
    double total_attack_best_speed_scale_095_control_smoothness_delta = 0.0;
    double total_attack_best_speed_scale_095_progress_delta = 0.0;
    double total_attack_best_speed_scale_095_primitive_bias_delta = 0.0;
    double total_phase_speed_bias = 0.0;
    double total_phase_margin_bias = 0.0;
    double total_phase_progress_bias = 0.0;
    double total_course_speed_error = 0.0;
    double total_abs_course_lateral_error = 0.0;
    double total_course_target_gap = 0.0;
    double total_dynamic_cap_gap = 0.0;
    double total_attack_residual_delta_v_ref = 0.0;
    double total_attack_residual_delta_steer_ff = 0.0;
    double total_attack_residual_delta_ax_bias = 0.0;
    double total_attack_residual_delta_heading_relax = 0.0;
    double total_attack_effective_heading_error = 0.0;
    double total_accel_headroom = 0.0;
    int healthy_underspeed_count = 0;
    int attack_residual_safety_gate_count = 0;
    int attack_best_tracking_cost_count = 0;
    int attack_best_boundary_recovery_cost_count = 0;
    int attack_best_low_speed_pure_pursuit_cost_count = 0;
    int attack_best_speed_scale_065_cost_count = 0;
    int attack_best_speed_scale_080_cost_count = 0;
    int attack_best_speed_scale_095_cost_count = 0;
    int attack_best_speed_scale_105_cost_count = 0;
    int attack_best_speed_scale_095_delta_count = 0;
    int attack_metric_count = 0;
    double min_mpc_predicted_min_margin = std::numeric_limits<double>::infinity();
    double min_steering_actuator_speed_cap = std::numeric_limits<double>::infinity();
    double min_steering_reach_speed_cap = std::numeric_limits<double>::infinity();
    double min_track_margin_speed_cap = std::numeric_limits<double>::infinity();
    int driver_metric_count = 0;
    int steering_saturation_count = 0;
    std::array<int, static_cast<std::size_t>(DriverSection::Count)> driver_section_counts{};
    std::array<int, static_cast<std::size_t>(DriverLoss::Count)> driver_loss_counts{};
    int diff_count = 0;
    double prev_steering = 0.0;
    double prev_throttle = 0.0;
    double prev_brake = 0.0;
    double offtrack_time = 0.0;
    bool first_step = true;
    vehicle::DriverInput rl_hold_input;
    bool rl_hold_valid = false;
    double last_rl_update_time = -1e9;
    vehicle::DriverInput mpc_hold_input;
    bool mpc_hold_valid = false;
    double last_mpc_update_time = -1e9;
    constexpr double kMpcControlPeriod = 0.025;
    double next_output_log_time = 0.0;

    // MAP座標での開始位置（座標系に応じて変換）
    double start_x, start_y, start_heading;
    start_x = vehicle->state().y;
    start_y = vehicle->state().x;
    start_heading = M_PI / 2.0 - vehicle->state().psi;
    double prev_x = start_x;
    double prev_y = start_y;
    double absolute_distance = 0.0;

    double finish_nx = std::cos(start_heading);
    double finish_ny = std::sin(start_heading);
    double prev_finish_dist = 0.0;
    int finish_line_crossings = 0;
    double start_track_width = start_point.width_left + start_point.width_right;
    double finish_capture_radius = std::max(10.0, 2.0 * start_track_width);
    double finish_centerline_tolerance = std::max(5.0, start_track_width);
    double finish_s_margin = std::max(10.0, 2.0 * start_track_width);
    // Circle / skidpad / autocross / CSV tracks: 1 crossing = 1 lap
    // Figure-eight: 2 crossings to ensure full figure traversal
    int required_crossings = ((config.track_name == "figure_eight") ? 2 : 1) * std::max(1, config.laps);

    const double stall_check_time = std::min(5.0, std::max(3.0, sim_time * 0.1));
    int loop_counter = 0;

    auto simple_driver_section = [&](double s) {
        constexpr double kCornerThreshold = 0.008;
        return std::abs(track->get_curvature(s)) < kCornerThreshold
            ? DriverSection::Straight
            : DriverSection::Apex;
    };

    auto attack_phase_section = [](const std::string& phase) {
        if (phase == "straight") {
            return DriverSection::Straight;
        }
        if (phase == "brake" || phase == "turn_in") {
            return DriverSection::Entry;
        }
        if (phase == "exit") {
            return DriverSection::Exit;
        }
        if (phase == "apex" || phase == "complex") {
            return DriverSection::Apex;
        }
        return DriverSection::Apex;
    };

    auto simple_mpc_loss = [&](const driver::MPCDebugInfo& debug) {
        if (debug.steering_actuator.steering_saturation > 0.5) {
            return DriverLoss::SteeringSaturation;
        }
        const double steering_cap_loss =
            normalized_speed_cap_loss(debug.steering_speed_cap, debug.target_speed_base);
        if (debug.steering_actuator.steering_lag_ratio > 0.08 || steering_cap_loss > 0.03) {
            return DriverLoss::SteeringLag;
        }
        if (debug.current_lateral_margin < 0.85) {
            return DriverLoss::TrackMargin;
        }
        return DriverLoss::None;
    };

    auto finalize_result_metrics = [&]() {
        if (sample_count > 0) {
            result.avg_lateral_error = total_lateral_error / sample_count;
            result.avg_speed = total_speed / sample_count;
            result.rms_heading_error = std::sqrt(total_heading_sq / sample_count);
        }
        if (diff_count > 0) {
            result.steering_smoothness = std::sqrt(total_steer_diff_sq / diff_count);
            result.throttle_smoothness = std::sqrt(total_thr_diff_sq / diff_count);
            result.brake_smoothness = std::sqrt(total_brake_diff_sq / diff_count);
        }
        if (driver_metric_count > 0) {
            result.mean_abs_sw_error = total_sw_error_abs / driver_metric_count;
            result.mean_steering_lag_ratio = total_steering_lag_ratio / driver_metric_count;
            result.avg_torque_speed_multiplier = total_torque_speed_multiplier / driver_metric_count;
            result.mean_torque_speed_loss = total_torque_speed_loss / driver_metric_count;
            result.mean_steering_cap_speed_loss = total_steering_cap_speed_loss / driver_metric_count;
            result.mean_steering_reach_speed_loss = total_steering_reach_speed_loss / driver_metric_count;
            result.mean_track_margin_speed_loss = total_track_margin_speed_loss / driver_metric_count;
            result.mean_corner_throttle_loss = total_corner_throttle_loss / driver_metric_count;
            result.mean_chassis_twist_risk = total_chassis_twist_risk / driver_metric_count;
            result.chassis_twist_risk_integral = total_chassis_twist_risk * config.dt;
            result.steering_saturation_ratio =
                static_cast<double>(steering_saturation_count) / driver_metric_count;
            result.control_intervention_cost =
                result.mean_torque_speed_loss
                + result.mean_steering_cap_speed_loss
                + result.mean_steering_reach_speed_loss
                + result.mean_track_margin_speed_loss
                + result.mean_corner_throttle_loss
                + result.steering_saturation_ratio;
            result.vehicle_health_penalty = result.mean_chassis_twist_risk;
            result.driver_masking_risk =
                result.vehicle_health_penalty
                * (result.mean_torque_speed_loss
                   + result.mean_track_margin_speed_loss
                   + result.mean_corner_throttle_loss);
            result.driver_intervention_cost =
                result.control_intervention_cost + result.vehicle_health_penalty;
            result.section_straight_ratio =
                static_cast<double>(driver_section_counts[static_cast<std::size_t>(DriverSection::Straight)]) / driver_metric_count;
            result.section_entry_ratio =
                static_cast<double>(driver_section_counts[static_cast<std::size_t>(DriverSection::Entry)]) / driver_metric_count;
            result.section_apex_ratio =
                static_cast<double>(driver_section_counts[static_cast<std::size_t>(DriverSection::Apex)]) / driver_metric_count;
            result.section_exit_ratio =
                static_cast<double>(driver_section_counts[static_cast<std::size_t>(DriverSection::Exit)]) / driver_metric_count;
            result.dominant_loss_none_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::None)]) / driver_metric_count;
            result.dominant_loss_steering_saturation_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::SteeringSaturation)]) / driver_metric_count;
            result.dominant_loss_steering_lag_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::SteeringLag)]) / driver_metric_count;
            result.dominant_loss_track_margin_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::TrackMargin)]) / driver_metric_count;
            result.dominant_loss_yaw_mismatch_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::YawMismatch)]) / driver_metric_count;
            result.dominant_loss_front_slip_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::FrontSlip)]) / driver_metric_count;
            result.dominant_loss_chassis_twist_ratio =
                static_cast<double>(driver_loss_counts[static_cast<std::size_t>(DriverLoss::ChassisTwist)]) / driver_metric_count;
            std::size_t dominant_loss_index = static_cast<std::size_t>(DriverLoss::None);
            int dominant_loss_count = driver_loss_counts[dominant_loss_index];
            for (std::size_t i = 1; i < driver_loss_counts.size(); ++i) {
                if (driver_loss_counts[i] > dominant_loss_count) {
                    dominant_loss_count = driver_loss_counts[i];
                    dominant_loss_index = i;
                }
            }
            result.dominant_loss = driver_loss_name(static_cast<DriverLoss>(dominant_loss_index));
            if (std::isfinite(min_steering_actuator_speed_cap)) {
                result.min_steering_actuator_speed_cap = min_steering_actuator_speed_cap;
            }
            if (std::isfinite(min_steering_reach_speed_cap)) {
                result.min_steering_reach_speed_cap = min_steering_reach_speed_cap;
            }
            if (std::isfinite(min_track_margin_speed_cap)) {
                result.min_track_margin_speed_cap = min_track_margin_speed_cap;
            }
        }
        if (mpc_progress_metric_count > 0) {
            result.mean_mpc_rollout_progress_deficit =
                total_mpc_rollout_progress_deficit / mpc_progress_metric_count;
            result.mean_mpc_predicted_min_margin =
                total_mpc_predicted_min_margin / mpc_progress_metric_count;
            result.mean_mpc_predicted_terminal_abs_n =
                total_mpc_predicted_terminal_abs_n / mpc_progress_metric_count;
            result.mean_mpc_predicted_terminal_abs_heading =
                total_mpc_predicted_terminal_abs_heading / mpc_progress_metric_count;
            result.mean_mpc_predicted_terminal_speed =
                total_mpc_predicted_terminal_speed / mpc_progress_metric_count;
            result.mean_mpc_predicted_signed_progress =
                total_mpc_predicted_signed_progress / mpc_progress_metric_count;
            if (std::isfinite(min_mpc_predicted_min_margin)) {
                result.min_mpc_predicted_min_margin = min_mpc_predicted_min_margin;
            }
        }
        if (attack_metric_count > 0) {
            result.mean_attack_cost_tracking =
                total_attack_cost_tracking / attack_metric_count;
            result.mean_attack_cost_speed =
                total_attack_cost_speed / attack_metric_count;
            result.mean_attack_cost_boundary =
                total_attack_cost_boundary / attack_metric_count;
            result.mean_attack_cost_control_smoothness =
                total_attack_cost_control_smoothness / attack_metric_count;
            result.mean_attack_cost_progress =
                total_attack_cost_progress / attack_metric_count;
            result.mean_attack_cost_primitive_bias =
                total_attack_cost_primitive_bias / attack_metric_count;
            result.mean_attack_cost_overspeed =
                total_attack_cost_overspeed / attack_metric_count;
            result.mean_attack_cost_underspeed =
                total_attack_cost_underspeed / attack_metric_count;
            result.mean_attack_cost_time_progress =
                total_attack_cost_time_progress / attack_metric_count;
            result.mean_attack_rollout_reject_count =
                total_attack_rollout_reject_count / attack_metric_count;
            result.mean_attack_rollout_boundary_reject_count =
                total_attack_rollout_boundary_reject_count / attack_metric_count;
            result.mean_attack_rollout_nonfinite_reject_count =
                total_attack_rollout_nonfinite_reject_count / attack_metric_count;
            result.mean_phase_speed_bias =
                total_phase_speed_bias / attack_metric_count;
            result.mean_phase_margin_bias =
                total_phase_margin_bias / attack_metric_count;
            result.mean_phase_progress_bias =
                total_phase_progress_bias / attack_metric_count;
            result.mean_course_speed_error =
                total_course_speed_error / attack_metric_count;
            result.mean_abs_course_lateral_error =
                total_abs_course_lateral_error / attack_metric_count;
            result.mean_course_target_gap =
                total_course_target_gap / attack_metric_count;
            result.mean_dynamic_cap_gap =
                total_dynamic_cap_gap / attack_metric_count;
            result.mean_attack_residual_delta_v_ref =
                total_attack_residual_delta_v_ref / attack_metric_count;
            result.mean_attack_residual_delta_steer_ff =
                total_attack_residual_delta_steer_ff / attack_metric_count;
            result.mean_attack_residual_delta_ax_bias =
                total_attack_residual_delta_ax_bias / attack_metric_count;
            result.mean_attack_residual_delta_heading_relax =
                total_attack_residual_delta_heading_relax / attack_metric_count;
            result.attack_residual_safety_gate_ratio =
                static_cast<double>(attack_residual_safety_gate_count) / attack_metric_count;
            result.mean_attack_effective_heading_error =
                total_attack_effective_heading_error / attack_metric_count;
            result.mean_accel_headroom =
                total_accel_headroom / attack_metric_count;
            result.healthy_underspeed_ratio =
                static_cast<double>(healthy_underspeed_count) / attack_metric_count;
            if (attack_best_tracking_cost_count > 0) {
                result.mean_attack_best_tracking_cost =
                    total_attack_best_tracking_cost / attack_best_tracking_cost_count;
            }
            if (attack_best_boundary_recovery_cost_count > 0) {
                result.mean_attack_best_boundary_recovery_cost =
                    total_attack_best_boundary_recovery_cost
                    / attack_best_boundary_recovery_cost_count;
            }
            if (attack_best_low_speed_pure_pursuit_cost_count > 0) {
                result.mean_attack_best_low_speed_pure_pursuit_cost =
                    total_attack_best_low_speed_pure_pursuit_cost
                    / attack_best_low_speed_pure_pursuit_cost_count;
            }
            if (attack_best_speed_scale_065_cost_count > 0) {
                result.mean_attack_best_speed_scale_065_cost =
                    total_attack_best_speed_scale_065_cost
                    / attack_best_speed_scale_065_cost_count;
            }
            if (attack_best_speed_scale_080_cost_count > 0) {
                result.mean_attack_best_speed_scale_080_cost =
                    total_attack_best_speed_scale_080_cost
                    / attack_best_speed_scale_080_cost_count;
            }
            if (attack_best_speed_scale_095_cost_count > 0) {
                result.mean_attack_best_speed_scale_095_cost =
                    total_attack_best_speed_scale_095_cost
                    / attack_best_speed_scale_095_cost_count;
            }
            if (attack_best_speed_scale_105_cost_count > 0) {
                result.mean_attack_best_speed_scale_105_cost =
                    total_attack_best_speed_scale_105_cost
                    / attack_best_speed_scale_105_cost_count;
            }
            if (attack_best_speed_scale_095_delta_count > 0) {
                result.mean_attack_best_speed_scale_095_cost_delta =
                    total_attack_best_speed_scale_095_cost_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_tracking_delta =
                    total_attack_best_speed_scale_095_tracking_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_speed_delta =
                    total_attack_best_speed_scale_095_speed_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_boundary_delta =
                    total_attack_best_speed_scale_095_boundary_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_control_smoothness_delta =
                    total_attack_best_speed_scale_095_control_smoothness_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_progress_delta =
                    total_attack_best_speed_scale_095_progress_delta
                    / attack_best_speed_scale_095_delta_count;
                result.mean_attack_best_speed_scale_095_primitive_bias_delta =
                    total_attack_best_speed_scale_095_primitive_bias_delta
                    / attack_best_speed_scale_095_delta_count;
            }
        }
        if ((use_lpv_lqr_driver && lpv_lqr_driver) ||
            (use_mpc_driver && mpc_driver) ||
            (use_mpc_v2_driver && mpc_v2_driver) ||
            (use_attack_driver && attack_driver)) {
            const auto& mpc_stats = use_lpv_lqr_driver
                ? lpv_lqr_driver->stats()
                : (use_attack_driver
                    ? attack_driver->stats()
                    : (use_mpc_v2_driver ? mpc_v2_driver->stats() : mpc_driver->stats()));
            result.mpc_solve_count       = mpc_stats.solve_count;
            result.mpc_avg_solve_time_ms = mpc_stats.avg_solve_time_ms();
            result.mpc_max_solve_time_ms = mpc_stats.max_solve_time_ms;
            result.mpc_fail_count        = mpc_stats.fail_count;
            if (use_lpv_lqr_driver) {
                result.actuator_prediction_count = mpc_stats.actuator_prediction_count;
                result.actuator_prediction_bias = mpc_stats.actuator_prediction_bias();
                result.actuator_prediction_mae = mpc_stats.actuator_prediction_mae();
                result.actuator_prediction_rmse = mpc_stats.actuator_prediction_rmse();
                result.actuator_prediction_max_abs_error =
                    mpc_stats.actuator_prediction_max_abs_error;
            }
        }
    };

    for (double t = 0.0; t < sim_time; t += config.dt) {
        if (loop_counter++ % 1000 == 0 && !config.output_json_result) {
            std::cout << "DEBUG: Simulation t=" << t << "s" << std::endl;
        }
        const auto state_has_nonfinite = [](const auto& state) {
            return !std::isfinite(state.x) || !std::isfinite(state.y) ||
                   !std::isfinite(state.psi) || !std::isfinite(state.vx) ||
                   !std::isfinite(state.vy) || !std::isfinite(state.vz) ||
                   !std::isfinite(state.p) || !std::isfinite(state.q) ||
                   !std::isfinite(state.r) || !std::isfinite(state.delta_sw) ||
                   !std::isfinite(state.ddelta_sw);
        };

        if (state_has_nonfinite(vehicle->state())) {
            std::cerr << "Physics diverged (NaN/Inf) at t = " << t << "s. Aborting simulation." << std::endl;
            fail_result(result, "physics_divergence",
                        "Vehicle state became non-finite before control update.", t);
            finalize_result_metrics();
            return result;
        }

        vehicle::DriverInput input;
        driver::PIDDebugInfo pid_debug;
        driver::MPCDebugInfo mpc_debug;
        double front_T_SAT_total = 0.0;
        if (vehicle_params.steering.torque_mode) {
            front_T_SAT_total = vehicle->tire_Mz()[vehicle::Vehicle14DOF_New::FL] +
                                vehicle->tire_Mz()[vehicle::Vehicle14DOF_New::FR];
        }
        if (use_rl_driver) {
            constexpr double kRlControlPeriod = 0.020;  // Match fsae_rl_server CONTROL_DT.
            if (!rl_hold_valid || (t - last_rl_update_time) >= (kRlControlPeriod - 1e-9)) {
                rl_hold_input = rl_driver->compute_control(vehicle->state(), *track, frenet_converter, t);
                rl_hold_valid = true;
                last_rl_update_time = t;
            }
            input = rl_hold_input;
            if (!driver::rl_driver_speed_limits_disabled()) {
                driver::apply_rl_curvature_speed_limit(vehicle->state(), *track, frenet_converter, input, vehicle_params, perf.ggv.get());
            }
            rl_driver->update_prev_action(input.steering, input.throttle, input.brake);
        } else if (use_predictive_driver) {
            if (!mpc_hold_valid || (t - last_mpc_update_time) >= (kMpcControlPeriod - 1e-9)) {
                if (vehicle_params.steering.torque_mode) {
                    if (use_lpv_lqr_driver) {
                        lpv_lqr_driver->set_steering_state(
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw);
                    } else if (use_attack_driver) {
                        attack_driver->set_steering_state(
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw);
                    } else if (use_mpc_v2_driver) {
                        mpc_v2_driver->set_steering_state(
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw);
                    } else {
                        mpc_driver->set_steering_state(
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw);
                    }
                }
                const auto map_state = driver::to_map_driver_state(vehicle->state());
                if (use_lpv_lqr_driver) {
                    mpc_hold_input = lpv_lqr_driver->compute_outer_control(map_state, t);
                } else if (use_attack_driver) {
                    mpc_hold_input = attack_driver->compute_outer_control(map_state, t);
                } else {
                    mpc_hold_input = use_mpc_v2_driver
                        ? mpc_v2_driver->compute_outer_control(map_state, t)
                        : mpc_driver->compute_outer_control(map_state, t);
                }
                mpc_hold_valid = true;
                last_mpc_update_time = t;
            }
            if (vehicle_params.steering.torque_mode) {
                if (use_lpv_lqr_driver) {
                    mpc_hold_input = lpv_lqr_driver->update_torque_actuator_control(
                        mpc_hold_input,
                        vehicle->state().delta_sw,
                        vehicle->state().ddelta_sw,
                        config.dt,
                        front_T_SAT_total);
                } else if (use_attack_driver) {
                    mpc_hold_input = attack_driver->update_torque_actuator_control(
                        mpc_hold_input,
                        vehicle->state().delta_sw,
                        vehicle->state().ddelta_sw,
                        config.dt,
                        front_T_SAT_total);
                } else {
                    mpc_hold_input = use_mpc_v2_driver
                        ? mpc_v2_driver->update_torque_actuator_control(
                            mpc_hold_input,
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw,
                            config.dt,
                            front_T_SAT_total)
                        : mpc_driver->update_torque_actuator_control(
                            mpc_hold_input,
                            vehicle->state().delta_sw,
                            vehicle->state().ddelta_sw,
                            config.dt,
                            front_T_SAT_total);
                }
            }
            input = mpc_hold_input;
            mpc_debug = use_lpv_lqr_driver
                ? lpv_lqr_driver->last_debug()
                : (use_attack_driver
                    ? attack_driver->last_debug()
                    : (use_mpc_v2_driver ? mpc_v2_driver->last_debug() : mpc_driver->last_debug()));
        } else {
            input = pid_driver->compute_control(vehicle->state(), t, front_T_SAT_total);
            pid_debug = pid_driver->get_last_debug();
        }

        if (teacher_ofs.is_open() && t + 1e-12 >= next_teacher_log_time) {
            vehicle::DriverInput teacher_input = input;
            if (!use_pid_driver && pid_driver) {
                double T_SAT_total = 0.0;
                if (vehicle_params.steering.torque_mode) {
                    T_SAT_total = vehicle->tire_Mz()[vehicle::Vehicle14DOF_New::FL] +
                                  vehicle->tire_Mz()[vehicle::Vehicle14DOF_New::FR];
                }
                teacher_input = pid_driver->compute_control(vehicle->state(), t, T_SAT_total);
            }
            const auto obs = build_rl_teacher_observation(
                vehicle->state(),
                *track,
                teacher_converter,
                teacher_prev_input,
                vehicle_params);
            write_rl_teacher_sample(teacher_ofs, t, obs, teacher_input, vehicle_params);
            teacher_prev_input = input;
            teacher_prev_input.steering = driver::smooth_rl_steering_command(
                input.steering,
                obs[6],
                driver::rl_steering_command_limit(vehicle_params),
                vehicle_params.steering.torque_mode);
            next_teacher_log_time += kTeacherControlPeriod;
        }
        vehicle->step_adaptive(config.dt, input);

        if (state_has_nonfinite(vehicle->state())) {
            std::cerr << "Physics diverged (NaN/Inf) at t = " << t << "s. Aborting simulation." << std::endl;
            fail_result(result, "physics_divergence",
                        "Vehicle state became non-finite after dynamics integration.", t);
            finalize_result_metrics();
            return result;
        }

        auto frenet = !use_pid_driver
            ? frenet_converter.global_to_frenet(
                vehicle->state().y,
                vehicle->state().x,
                M_PI / 2.0 - vehicle->state().psi)
            : pid_driver->get_current_frenet(vehicle->state());

        if (!std::isfinite(frenet.s) || !std::isfinite(frenet.n) || !std::isfinite(frenet.d_heading)) {
            fail_result(result, "physics_divergence",
                        "Frenet projection produced non-finite values.", t);
            finalize_result_metrics();
            return result;
        }

        // MAP座標に変換（ラップ検出・距離計算用）
        double map_x = vehicle->state().y;
        double map_y = vehicle->state().x;

        if (t > 0.0) {
            double dx = map_x - prev_x;
            double dy = map_y - prev_y;
            absolute_distance += std::sqrt(dx * dx + dy * dy);
            prev_x = map_x;
            prev_y = map_y;
        }

        double finish_dist = (map_x - start_x) * finish_nx + (map_y - start_y) * finish_ny;

        if (!result.lap_completed) {
            if (config.track_name == "acceleration" || config.track_name == "accel") {
                if (t > 0.5 && finish_dist >= track->length() * std::max(1, config.laps)) {
                    result.lap_completed = true;
                    result.lap_time = t;
                    result.distance_traveled = absolute_distance;
                }
            } else {
                if (t > 0.5 && prev_finish_dist < 0.0 && finish_dist >= 0.0) {
                    double dx_start = map_x - start_x;
                    double dy_start = map_y - start_y;
                    double dist_to_start = std::sqrt(dx_start * dx_start + dy_start * dy_start);
                    bool near_finish_s = frenet.s <= finish_s_margin || frenet.s >= (track->length() - finish_s_margin);
                    bool near_centerline = std::abs(frenet.n) <= finish_centerline_tolerance;

                    if (dist_to_start < finish_capture_radius &&
                        near_finish_s &&
                        near_centerline &&
                        absolute_distance > track->length() * 0.8) {
                        finish_line_crossings++;
                        if (finish_line_crossings >= required_crossings) {
                            result.lap_completed = true;
                            result.lap_time = t;
                            result.distance_traveled = absolute_distance;
                        }
                    }
                }
            }
        }
        prev_finish_dist = finish_dist;

        if (result.lap_completed) break;

        if (!std::isnan(absolute_distance) && !std::isinf(absolute_distance)) {
            result.distance_traveled = absolute_distance;
        }

        double lateral_error = std::abs(frenet.n);
        const auto current_track_point = track->get_point(frenet.s);
        const double lateral_limit = (frenet.n >= 0.0)
            ? current_track_point.width_left
            : current_track_point.width_right;
        const double track_limit_excess = lateral_error - lateral_limit;
        result.max_track_limit_excess = std::max(result.max_track_limit_excess, track_limit_excess);
        if (track_limit_excess > 0.10) {
            offtrack_time += config.dt;
        } else {
            offtrack_time = 0.0;
        }
        if (offtrack_time > 0.25) {
            std::ostringstream oss;
            oss << "Vehicle left the track for " << offtrack_time
                << " s; lateral_error=" << lateral_error
                << " m, limit=" << lateral_limit << " m.";
            fail_result(result, "off_track", oss.str(), t);
            finalize_result_metrics();
            return result;
        }

        double cur_speed = vehicle->state().speed();
        if (!std::isnan(lateral_error) && !std::isinf(lateral_error)) {
            total_lateral_error += lateral_error;
            result.max_lateral_error = std::max(result.max_lateral_error, lateral_error);
        }
        if (!std::isnan(cur_speed) && !std::isinf(cur_speed)) {
            total_speed += cur_speed;
            result.max_speed = std::max(result.max_speed, cur_speed);
        }
        const auto st_metrics = vehicle->state();
        result.max_abs_phi_twist = std::max(result.max_abs_phi_twist, std::abs(st_metrics.phi_twist));
        result.max_abs_dphi_twist = std::max(result.max_abs_dphi_twist, std::abs(st_metrics.dphi_twist));
        if ((use_pid_driver || use_predictive_driver) && vehicle_params.steering.torque_mode) {
            const bool use_mpc_metrics = use_predictive_driver;
            const auto& actuator = mpc_debug.steering_actuator;

            const double sw_error = use_mpc_metrics ? actuator.sw_error : pid_debug.sw_error;
            const double steering_lag_ratio = use_mpc_metrics
                ? actuator.steering_lag_ratio
                : pid_debug.steering_lag_ratio;
            const double steering_saturation = use_mpc_metrics
                ? actuator.steering_saturation
                : pid_debug.steering_saturation;
            const double target_speed_base = use_mpc_metrics
                ? mpc_debug.target_speed_base
                : pid_debug.target_speed_preview;
            const double target_speed_final = use_mpc_metrics
                ? mpc_debug.target_speed
                : pid_debug.target_speed_final;
            const double torque_speed_multiplier = target_speed_base > 1e-6
                ? std::clamp(target_speed_final / target_speed_base, 0.0, 1.0)
                : 1.0;
            const double steering_speed_cap = use_mpc_metrics
                ? mpc_debug.steering_speed_cap
                : pid_debug.steering_actuator_speed_cap;
            const double track_margin_speed_cap = use_mpc_metrics
                ? mpc_debug.track_margin_speed_cap
                : pid_debug.track_margin_speed_cap;
            const double chassis_twist_risk = use_mpc_metrics
                ? std::clamp(
                    std::max(std::abs(st_metrics.phi_twist) / 0.04,
                             std::abs(st_metrics.dphi_twist) / 1.0),
                    0.0,
                    1.0)
                : pid_debug.chassis_twist_risk;

            const double sw_error_abs = std::abs(sw_error);
            total_sw_error_abs += sw_error_abs;
            result.max_abs_sw_error = std::max(result.max_abs_sw_error, sw_error_abs);
            total_steering_lag_ratio += steering_lag_ratio;
            result.max_steering_lag_ratio = std::max(result.max_steering_lag_ratio, steering_lag_ratio);
            total_torque_speed_multiplier += torque_speed_multiplier;
            total_torque_speed_loss += std::clamp(1.0 - torque_speed_multiplier, 0.0, 1.0);
            total_steering_cap_speed_loss += normalized_speed_cap_loss(
                steering_speed_cap,
                target_speed_base);
            if (!use_mpc_metrics) {
                total_steering_reach_speed_loss += normalized_speed_cap_loss(
                    pid_debug.steering_reach_speed_cap,
                    pid_debug.target_speed_preview);
            }
            total_track_margin_speed_loss += normalized_speed_cap_loss(
                track_margin_speed_cap,
                target_speed_base);
            if (!use_mpc_metrics) {
                total_corner_throttle_loss += std::clamp(1.0 - pid_debug.corner_throttle_scale, 0.0, 1.0);
            }
            total_chassis_twist_risk += std::clamp(chassis_twist_risk, 0.0, 1.0);
            if (std::isfinite(steering_speed_cap)) {
                min_steering_actuator_speed_cap =
                    std::min(min_steering_actuator_speed_cap, steering_speed_cap);
            }
            if (!use_mpc_metrics && std::isfinite(pid_debug.steering_reach_speed_cap)) {
                min_steering_reach_speed_cap =
                    std::min(min_steering_reach_speed_cap, pid_debug.steering_reach_speed_cap);
            }
            if (std::isfinite(track_margin_speed_cap)) {
                min_track_margin_speed_cap =
                    std::min(min_track_margin_speed_cap, track_margin_speed_cap);
            }
            if (steering_saturation > 0.5) {
                steering_saturation_count++;
            }
            const DriverSection section = use_attack_driver
                ? attack_phase_section(mpc_debug.track_phase)
                : (use_mpc_metrics
                    ? simple_driver_section(frenet.s)
                    : classify_driver_section(pid_debug));
            const DriverLoss loss = use_mpc_metrics
                ? simple_mpc_loss(mpc_debug)
                : classify_dominant_driver_loss(pid_debug);
            driver_section_counts[static_cast<std::size_t>(section)]++;
            driver_loss_counts[static_cast<std::size_t>(loss)]++;
            driver_metric_count++;
        }
        if (use_predictive_driver) {
            total_mpc_rollout_progress_deficit += mpc_debug.rollout_progress_deficit;
            total_mpc_predicted_min_margin += mpc_debug.predicted_min_margin;
            total_mpc_predicted_terminal_abs_n += std::abs(mpc_debug.predicted_terminal_n);
            total_mpc_predicted_terminal_abs_heading +=
                std::abs(mpc_debug.predicted_terminal_heading);
            total_mpc_predicted_terminal_speed += mpc_debug.predicted_terminal_speed;
            total_mpc_predicted_signed_progress += mpc_debug.predicted_signed_progress;
            result.max_mpc_rollout_progress_deficit = std::max(
                result.max_mpc_rollout_progress_deficit,
                mpc_debug.rollout_progress_deficit);
            if (std::isfinite(mpc_debug.predicted_min_margin)) {
                min_mpc_predicted_min_margin =
                    std::min(min_mpc_predicted_min_margin, mpc_debug.predicted_min_margin);
            }
            result.last_mpc_selected_plan_index = mpc_debug.selected_plan_index;
            mpc_progress_metric_count++;
        }
        if (use_attack_driver) {
            total_attack_cost_tracking += mpc_debug.attack_cost_tracking;
            total_attack_cost_speed += mpc_debug.attack_cost_speed;
            total_attack_cost_boundary += mpc_debug.attack_cost_boundary;
            total_attack_cost_control_smoothness +=
                mpc_debug.attack_cost_control_smoothness;
            total_attack_cost_progress += mpc_debug.attack_cost_progress;
            total_attack_cost_primitive_bias += mpc_debug.attack_cost_primitive_bias;
            total_attack_cost_overspeed += mpc_debug.attack_cost_overspeed;
            total_attack_cost_underspeed += mpc_debug.attack_cost_underspeed;
            total_attack_cost_time_progress += mpc_debug.attack_cost_time_progress;
            total_phase_speed_bias += mpc_debug.phase_speed_bias;
            total_phase_margin_bias += mpc_debug.phase_margin_bias;
            total_phase_progress_bias += mpc_debug.phase_progress_bias;
            total_course_speed_error += mpc_debug.course_speed_error;
            total_abs_course_lateral_error += std::abs(mpc_debug.course_lateral_error);
            total_course_target_gap += mpc_debug.course_target_gap;
            total_dynamic_cap_gap += mpc_debug.dynamic_cap_gap;
            total_attack_residual_delta_v_ref += mpc_debug.attack_residual_delta_v_ref;
            total_attack_residual_delta_steer_ff += mpc_debug.attack_residual_delta_steer_ff;
            total_attack_residual_delta_ax_bias += mpc_debug.attack_residual_delta_ax_bias;
            total_attack_residual_delta_heading_relax +=
                mpc_debug.attack_residual_delta_heading_relax;
            total_attack_effective_heading_error += mpc_debug.attack_effective_heading_error;
            if (mpc_debug.attack_residual_safety_gate_active) {
                attack_residual_safety_gate_count++;
            }
            total_accel_headroom += mpc_debug.accel_headroom;
            if (mpc_debug.healthy_underspeed) {
                healthy_underspeed_count++;
            }
            total_attack_rollout_reject_count += mpc_debug.attack_rollout_reject_count;
            total_attack_rollout_boundary_reject_count +=
                mpc_debug.attack_rollout_boundary_reject_count;
            total_attack_rollout_nonfinite_reject_count +=
                mpc_debug.attack_rollout_nonfinite_reject_count;
            result.max_attack_rollout_reject_count = std::max(
                result.max_attack_rollout_reject_count,
                mpc_debug.attack_rollout_reject_count);
            if (std::isfinite(mpc_debug.attack_best_tracking_cost)) {
                total_attack_best_tracking_cost += mpc_debug.attack_best_tracking_cost;
                attack_best_tracking_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_boundary_recovery_cost)) {
                total_attack_best_boundary_recovery_cost +=
                    mpc_debug.attack_best_boundary_recovery_cost;
                attack_best_boundary_recovery_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_low_speed_pure_pursuit_cost)) {
                total_attack_best_low_speed_pure_pursuit_cost +=
                    mpc_debug.attack_best_low_speed_pure_pursuit_cost;
                attack_best_low_speed_pure_pursuit_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_speed_scale_065_cost)) {
                total_attack_best_speed_scale_065_cost +=
                    mpc_debug.attack_best_speed_scale_065_cost;
                attack_best_speed_scale_065_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_speed_scale_080_cost)) {
                total_attack_best_speed_scale_080_cost +=
                    mpc_debug.attack_best_speed_scale_080_cost;
                attack_best_speed_scale_080_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_speed_scale_095_cost)) {
                total_attack_best_speed_scale_095_cost +=
                    mpc_debug.attack_best_speed_scale_095_cost;
                attack_best_speed_scale_095_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_speed_scale_105_cost)) {
                total_attack_best_speed_scale_105_cost +=
                    mpc_debug.attack_best_speed_scale_105_cost;
                attack_best_speed_scale_105_cost_count++;
            }
            if (std::isfinite(mpc_debug.attack_best_speed_scale_095_cost_delta)) {
                total_attack_best_speed_scale_095_cost_delta +=
                    mpc_debug.attack_best_speed_scale_095_cost_delta;
                total_attack_best_speed_scale_095_tracking_delta +=
                    mpc_debug.attack_best_speed_scale_095_tracking_delta;
                total_attack_best_speed_scale_095_speed_delta +=
                    mpc_debug.attack_best_speed_scale_095_speed_delta;
                total_attack_best_speed_scale_095_boundary_delta +=
                    mpc_debug.attack_best_speed_scale_095_boundary_delta;
                total_attack_best_speed_scale_095_control_smoothness_delta +=
                    mpc_debug.attack_best_speed_scale_095_control_smoothness_delta;
                total_attack_best_speed_scale_095_progress_delta +=
                    mpc_debug.attack_best_speed_scale_095_progress_delta;
                total_attack_best_speed_scale_095_primitive_bias_delta +=
                    mpc_debug.attack_best_speed_scale_095_primitive_bias_delta;
                attack_best_speed_scale_095_delta_count++;
            }
            result.last_attack_selected_primitive =
                mpc_debug.attack_selected_primitive;
            result.last_attack_selected_primitive_kind =
                mpc_debug.attack_selected_primitive_kind;
            result.last_attack_selected_speed_scale =
                mpc_debug.attack_selected_speed_scale;
            result.last_attack_selected_target_speed =
                mpc_debug.attack_selected_target_speed;
            result.last_attack_selected_ax =
                mpc_debug.attack_selected_ax;
            result.last_course_v_ref = mpc_debug.course_v_ref;
            result.last_course_n_ref = mpc_debug.course_n_ref;
            result.last_course_kappa_ref = mpc_debug.course_kappa_ref;
            result.last_course_a_ref = mpc_debug.course_a_ref;
            result.last_attack_residual_delta_v_ref = mpc_debug.attack_residual_delta_v_ref;
            result.last_attack_residual_delta_steer_ff = mpc_debug.attack_residual_delta_steer_ff;
            result.last_attack_residual_delta_ax_bias =
                mpc_debug.attack_residual_delta_ax_bias;
            result.last_attack_residual_delta_heading_relax =
                mpc_debug.attack_residual_delta_heading_relax;
            result.last_attack_residual_safety_gate_active =
                mpc_debug.attack_residual_safety_gate_active;
            result.last_attack_raw_heading_error = mpc_debug.attack_raw_heading_error;
            result.last_attack_effective_heading_error =
                mpc_debug.attack_effective_heading_error;
            result.last_attack_ax_before_residual = mpc_debug.attack_ax_before_residual;
            result.last_attack_ax_after_residual = mpc_debug.attack_ax_after_residual;
            result.last_longitudinal_mode = mpc_debug.longitudinal_mode;
            result.last_track_phase = mpc_debug.track_phase;
            result.last_attack_segment_id = mpc_debug.attack_segment_id;
            result.last_attack_allowed = mpc_debug.attack_allowed;
            result.last_attack_static_speed_cap = mpc_debug.attack_static_speed_cap;
            result.last_attack_dynamic_speed_cap = mpc_debug.attack_dynamic_speed_cap;
            result.last_attack_all_rejected = mpc_debug.attack_all_rejected;
            attack_metric_count++;
        }
        double heading_deg = frenet.d_heading * 180.0 / M_PI;
        if (!std::isnan(heading_deg) && !std::isinf(heading_deg)) {
            total_heading_sq += heading_deg * heading_deg;
        }
        if (!first_step) {
            double ds = input.steering - prev_steering;
            double dt_thr = input.throttle - prev_throttle;
            double db = input.brake - prev_brake;
            if (!std::isnan(ds) && !std::isinf(ds)) total_steer_diff_sq += ds * ds;
            if (!std::isnan(dt_thr) && !std::isinf(dt_thr)) total_thr_diff_sq += dt_thr * dt_thr;
            if (!std::isnan(db) && !std::isinf(db)) total_brake_diff_sq += db * db;
            diff_count++;
        }
        prev_steering = input.steering;
        prev_throttle = input.throttle;
        prev_brake = input.brake;
        first_step = false;
        sample_count++;

        if (!result.lap_completed &&
            t >= stall_check_time &&
            absolute_distance < 3.0 &&
            result.max_speed < 1.0 &&
            cur_speed < 0.5) {
            std::ostringstream oss;
            oss << "Vehicle made insufficient progress: distance=" << absolute_distance
                << " m, max_speed=" << result.max_speed
                << " m/s by t=" << t << " s.";
            fail_result(result, "early_stall", oss.str(), t);
            finalize_result_metrics();
            return result;
        }

        // CSV出力（常にMAP座標で出力）
        if (ofs.is_open() && t + 0.5 * config.dt >= next_output_log_time) {
            auto s = vehicle->state();
            double target_speed_log = s.speed();
            if (use_pid_driver) {
                target_speed_log = pid_debug.target_speed_final;
            } else if (use_predictive_driver) {
                target_speed_log = mpc_debug.target_speed;
            }
            const bool log_mpc = use_predictive_driver;
            const bool log_mpc_torque_metrics = log_mpc && vehicle_params.steering.torque_mode;
            const auto& mpc_actuator = mpc_debug.steering_actuator;

            const double delta_sw_log = use_pid_driver ? pid_debug.delta_sw : s.delta_sw;
            const double ddelta_sw_log = use_pid_driver ? pid_debug.ddelta_sw : s.ddelta_sw;
            const double phi_twist_log = use_pid_driver ? pid_debug.phi_twist : s.phi_twist;
            const double dphi_twist_log = use_pid_driver ? pid_debug.dphi_twist : s.dphi_twist;
            const double target_speed_base_log = log_mpc
                ? mpc_debug.target_speed_base
                : pid_debug.target_speed_base;
            const double target_speed_preview_log = log_mpc
                ? mpc_debug.target_speed_base
                : pid_debug.target_speed_preview;
            const double target_speed_final_log = log_mpc
                ? mpc_debug.target_speed
                : pid_debug.target_speed_final;
            const double delta_target_log = log_mpc
                ? mpc_debug.target_tire_angle
                : pid_debug.delta_target;
            const double target_sw_log = log_mpc_torque_metrics
                ? mpc_actuator.target_sw
                : pid_debug.target_sw;
            const double actual_delta_tire_log = log_mpc_torque_metrics
                ? mpc_actuator.actual_delta_tire
                : pid_debug.actual_delta_tire;
            const double required_delta_preview_log = use_lpv_lqr_driver
                ? mpc_debug.required_tire_angle
                : pid_debug.required_delta_preview;
            const double predicted_delta_tire_log = use_lpv_lqr_driver
                ? mpc_debug.predicted_tire_angle
                : pid_debug.predicted_delta_tire;
            const double preview_delta_gap_log = use_lpv_lqr_driver
                ? mpc_debug.required_tire_angle - actual_delta_tire_log
                : pid_debug.preview_delta_gap;
            const double raw_delta_target_log = use_lpv_lqr_driver
                ? mpc_debug.raw_target_tire_angle
                : delta_target_log;
            const double inner_target_delta_tire_log = log_mpc_torque_metrics
                ? mpc_actuator.target_sw / std::max(vehicle_params.steering.R_steer, 1e-6)
                : pid_debug.target_sw / std::max(vehicle_params.steering.R_steer, 1e-6);
            const double lpv_prediction_error_log = use_lpv_lqr_driver
                ? mpc_debug.steering_prediction_error
                : 0.0;
            const double actual_delta_tire_rate_pre_log = use_lpv_lqr_driver
                ? mpc_debug.actual_tire_rate_pre
                : 0.0;
            const double predicted_delta_tire_rate_log = use_lpv_lqr_driver
                ? mpc_debug.predicted_tire_rate
                : 0.0;
            const double steering_prediction_horizon_log = use_lpv_lqr_driver
                ? mpc_debug.steering_prediction_horizon
                : 0.0;
            const double steering_prediction_target_time_log = use_lpv_lqr_driver
                ? mpc_debug.steering_prediction_target_time
                : 0.0;
            const int steering_prediction_valid_log =
                use_lpv_lqr_driver && mpc_debug.steering_prediction_valid ? 1 : 0;
            const int steering_prediction_error_valid_log =
                use_lpv_lqr_driver && mpc_debug.steering_prediction_error_valid ? 1 : 0;
            const auto lpv_outer_sequence_log = use_lpv_lqr_driver
                ? mpc_debug.outer_sequence
                : 0;
            const double required_reference_s_log = use_lpv_lqr_driver
                ? mpc_debug.required_reference_s
                : 0.0;
            const double required_reference_time_log = use_lpv_lqr_driver
                ? mpc_debug.required_reference_time
                : 0.0;
            const double model_yaw_rate_map_log = use_lpv_lqr_driver
                ? mpc_debug.model_yaw_rate
                : 0.0;
            const double actual_yaw_rate_map_log = use_lpv_lqr_driver
                ? -s.r
                : 0.0;
            const double yaw_rate_error_map_log = use_lpv_lqr_driver
                ? -s.r - mpc_debug.model_yaw_rate
                : 0.0;
            const double sw_error_log = log_mpc_torque_metrics
                ? mpc_actuator.sw_error
                : pid_debug.sw_error;
            const double torque_lag_lead_sw_log = log_mpc_torque_metrics
                ? mpc_actuator.lag_lead_sw
                : pid_debug.torque_lag_lead_sw;
            const double T_driver_raw_log = log_mpc_torque_metrics
                ? mpc_actuator.T_driver_raw
                : pid_debug.T_driver_raw;
            const double T_driver_clamped_log = log_mpc_torque_metrics
                ? mpc_actuator.T_driver_clamped
                : pid_debug.T_driver_clamped;
            const double torque_headroom_log = log_mpc_torque_metrics
                ? mpc_actuator.torque_headroom
                : pid_debug.torque_headroom;
            const double T_SAT_total_log = use_pid_driver
                ? pid_debug.T_SAT_total
                : (use_attack_driver || use_mpc_v2_driver || use_mpc_driver ? front_T_SAT_total : 0.0);
            const double T_SAT_reflected_log = T_SAT_total_log / std::max(vehicle_params.steering.R_steer, 1e-6);
            const double T_SAT_feedforward_log =
                use_pid_driver ? pid_debug.T_SAT_feedforward : mpc_actuator.T_SAT_feedforward;
            const double steering_saturation_log = log_mpc_torque_metrics
                ? mpc_actuator.steering_saturation
                : pid_debug.steering_saturation;
            const double torque_speed_multiplier_log =
                log_mpc && mpc_debug.target_speed_base > 1e-6
                    ? std::clamp(mpc_debug.target_speed / mpc_debug.target_speed_base, 0.0, 1.0)
                    : pid_debug.torque_speed_multiplier;
            const double steering_lag_ratio_log = log_mpc_torque_metrics
                ? mpc_actuator.steering_lag_ratio
                : pid_debug.steering_lag_ratio;
            const double effective_max_tire_steer_rate_log = log_mpc
                ? mpc_debug.effective_tire_steer_rate
                : pid_debug.effective_max_tire_steer_rate;
            const double steering_actuator_speed_cap_log = log_mpc
                ? mpc_debug.steering_speed_cap
                : pid_debug.steering_actuator_speed_cap;
            const double track_margin_speed_cap_log = log_mpc
                ? mpc_debug.track_margin_speed_cap
                : pid_debug.track_margin_speed_cap;
            const double mpc_rollout_progress_log = log_mpc ? mpc_debug.rollout_progress : 0.0;
            const double mpc_rollout_progress_target_log = log_mpc
                ? mpc_debug.rollout_progress_target
                : 0.0;
            const double mpc_rollout_progress_deficit_log = log_mpc
                ? mpc_debug.rollout_progress_deficit
                : 0.0;
            const double mpc_rollout_cost_log = log_mpc ? mpc_debug.rollout_cost : 0.0;
            const int mpc_selected_plan_index_log = log_mpc ? mpc_debug.selected_plan_index : -1;
            const double mpc_predicted_terminal_n_log = log_mpc ? mpc_debug.predicted_terminal_n : 0.0;
            const double mpc_predicted_terminal_heading_log =
                log_mpc ? mpc_debug.predicted_terminal_heading : 0.0;
            const double mpc_predicted_min_margin_log =
                log_mpc ? mpc_debug.predicted_min_margin : 0.0;
            const double mpc_predicted_terminal_speed_log =
                log_mpc ? mpc_debug.predicted_terminal_speed : 0.0;
            const double mpc_predicted_signed_progress_log =
                log_mpc ? mpc_debug.predicted_signed_progress : 0.0;
            const bool log_attack = use_attack_driver;
            const int attack_selected_primitive_log =
                log_attack ? mpc_debug.attack_selected_primitive : -1;
            const std::string attack_selected_primitive_kind_log =
                log_attack ? mpc_debug.attack_selected_primitive_kind : "unknown";
            const double attack_selected_speed_scale_log =
                log_attack ? mpc_debug.attack_selected_speed_scale : 0.0;
            const double attack_selected_target_speed_log =
                log_attack ? mpc_debug.attack_selected_target_speed : 0.0;
            const double attack_selected_ax_log =
                log_attack ? mpc_debug.attack_selected_ax : 0.0;
            const double course_v_ref_log =
                log_attack ? mpc_debug.course_v_ref : 0.0;
            const double course_n_ref_log =
                log_attack ? mpc_debug.course_n_ref : 0.0;
            const double course_kappa_ref_log =
                log_attack ? mpc_debug.course_kappa_ref : 0.0;
            const double course_speed_error_log =
                log_attack ? mpc_debug.course_speed_error : 0.0;
            const double course_lateral_error_log =
                log_attack ? mpc_debug.course_lateral_error : 0.0;
            const double course_a_ref_log =
                log_attack ? mpc_debug.course_a_ref : 0.0;
            const double course_target_gap_log =
                log_attack ? mpc_debug.course_target_gap : 0.0;
            const double dynamic_cap_gap_log =
                log_attack ? mpc_debug.dynamic_cap_gap : 0.0;
            const int braking_corridor_active_log =
                log_attack && mpc_debug.braking_corridor_active ? 1 : 0;
            const double braking_corridor_v_lower_log =
                log_attack ? mpc_debug.braking_corridor_v_lower : 0.0;
            const double braking_corridor_v_upper_log =
                log_attack ? mpc_debug.braking_corridor_v_upper : 0.0;
            const double attack_residual_delta_v_ref_log =
                log_attack ? mpc_debug.attack_residual_delta_v_ref : 0.0;
            const double attack_residual_delta_steer_ff_log =
                log_attack ? mpc_debug.attack_residual_delta_steer_ff : 0.0;
            const double attack_residual_delta_ax_bias_log =
                log_attack ? mpc_debug.attack_residual_delta_ax_bias : 0.0;
            const double attack_residual_delta_heading_relax_log =
                log_attack ? mpc_debug.attack_residual_delta_heading_relax : 0.0;
            const int attack_residual_safety_gate_active_log =
                log_attack && mpc_debug.attack_residual_safety_gate_active ? 1 : 0;
            const double attack_raw_heading_error_log =
                log_attack ? mpc_debug.attack_raw_heading_error : 0.0;
            const double attack_effective_heading_error_log =
                log_attack ? mpc_debug.attack_effective_heading_error : 0.0;
            const double attack_ax_before_residual_log =
                log_attack ? mpc_debug.attack_ax_before_residual : 0.0;
            const double attack_ax_after_residual_log =
                log_attack ? mpc_debug.attack_ax_after_residual : 0.0;
            const std::string longitudinal_mode_log =
                log_attack ? mpc_debug.longitudinal_mode : "unknown";
            const int healthy_underspeed_log =
                log_attack && mpc_debug.healthy_underspeed ? 1 : 0;
            const double accel_headroom_log =
                log_attack ? mpc_debug.accel_headroom : 0.0;
            const std::string track_phase_log =
                log_attack ? mpc_debug.track_phase : "unknown";
            const double phase_speed_bias_log =
                log_attack ? mpc_debug.phase_speed_bias : 1.0;
            const double phase_margin_bias_log =
                log_attack ? mpc_debug.phase_margin_bias : 0.0;
            const double phase_progress_bias_log =
                log_attack ? mpc_debug.phase_progress_bias : 1.0;
            const int attack_segment_id_log =
                log_attack ? mpc_debug.attack_segment_id : -1;
            const int attack_allowed_log =
                log_attack && mpc_debug.attack_allowed ? 1 : 0;
            const double attack_static_speed_cap_log =
                log_attack ? mpc_debug.attack_static_speed_cap : 0.0;
            const double attack_dynamic_speed_cap_log =
                log_attack ? mpc_debug.attack_dynamic_speed_cap : 0.0;
            const double attack_dynamic_base_speed_log =
                log_attack ? mpc_debug.attack_dynamic_base_speed : 0.0;
            const double attack_curvature_speed_cap_log =
                log_attack ? mpc_debug.attack_curvature_speed_cap : 0.0;
            const double attack_steering_speed_cap_log =
                log_attack ? mpc_debug.attack_steering_speed_cap : 0.0;
            const double attack_margin_speed_cap_log =
                log_attack ? mpc_debug.attack_margin_speed_cap : 0.0;
            const double attack_heading_speed_cap_log =
                log_attack ? mpc_debug.attack_heading_speed_cap : 0.0;
            const double attack_outward_speed_cap_log =
                log_attack ? mpc_debug.attack_outward_speed_cap : 0.0;
            const std::string attack_limiting_cap_reason_log =
                log_attack ? mpc_debug.attack_limiting_cap_reason : "unknown";
            const int attack_rollout_candidate_count_log =
                log_attack ? mpc_debug.attack_rollout_candidate_count : 0;
            const int attack_rollout_reject_count_log =
                log_attack ? mpc_debug.attack_rollout_reject_count : 0;
            const int attack_rollout_boundary_reject_count_log =
                log_attack ? mpc_debug.attack_rollout_boundary_reject_count : 0;
            const int attack_rollout_nonfinite_reject_count_log =
                log_attack ? mpc_debug.attack_rollout_nonfinite_reject_count : 0;
            const int attack_all_rejected_log =
                log_attack && mpc_debug.attack_all_rejected ? 1 : 0;
            const double attack_cost_tracking_log =
                log_attack ? mpc_debug.attack_cost_tracking : 0.0;
            const double attack_cost_speed_log =
                log_attack ? mpc_debug.attack_cost_speed : 0.0;
            const double attack_cost_boundary_log =
                log_attack ? mpc_debug.attack_cost_boundary : 0.0;
            const double attack_cost_control_smoothness_log =
                log_attack ? mpc_debug.attack_cost_control_smoothness : 0.0;
            const double attack_cost_progress_log =
                log_attack ? mpc_debug.attack_cost_progress : 0.0;
            const double attack_cost_primitive_bias_log =
                log_attack ? mpc_debug.attack_cost_primitive_bias : 0.0;
            const double attack_cost_overspeed_log =
                log_attack ? mpc_debug.attack_cost_overspeed : 0.0;
            const double attack_cost_underspeed_log =
                log_attack ? mpc_debug.attack_cost_underspeed : 0.0;
            const double attack_cost_time_progress_log =
                log_attack ? mpc_debug.attack_cost_time_progress : 0.0;
            const double attack_best_tracking_cost_log =
                log_attack ? mpc_debug.attack_best_tracking_cost : 0.0;
            const double attack_best_boundary_recovery_cost_log =
                log_attack ? mpc_debug.attack_best_boundary_recovery_cost : 0.0;
            const double attack_best_low_speed_pure_pursuit_cost_log =
                log_attack ? mpc_debug.attack_best_low_speed_pure_pursuit_cost : 0.0;
            const double attack_best_speed_scale_065_cost_log =
                log_attack ? mpc_debug.attack_best_speed_scale_065_cost : 0.0;
            const double attack_best_speed_scale_080_cost_log =
                log_attack ? mpc_debug.attack_best_speed_scale_080_cost : 0.0;
            const double attack_best_speed_scale_095_cost_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_cost : 0.0;
            const double attack_best_speed_scale_105_cost_log =
                log_attack ? mpc_debug.attack_best_speed_scale_105_cost : 0.0;
            const double attack_best_speed_scale_095_cost_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_cost_delta : 0.0;
            const double attack_best_speed_scale_095_tracking_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_tracking_delta : 0.0;
            const double attack_best_speed_scale_095_speed_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_speed_delta : 0.0;
            const double attack_best_speed_scale_095_boundary_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_boundary_delta : 0.0;
            const double attack_best_speed_scale_095_control_smoothness_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_control_smoothness_delta : 0.0;
            const double attack_best_speed_scale_095_progress_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_progress_delta : 0.0;
            const double attack_best_speed_scale_095_primitive_bias_delta_log =
                log_attack ? mpc_debug.attack_best_speed_scale_095_primitive_bias_delta : 0.0;
            const double chassis_twist_risk_log = log_mpc
                ? std::clamp(
                    std::max(std::abs(s.phi_twist) / 0.04, std::abs(s.dphi_twist) / 1.0),
                    0.0,
                    1.0)
                : pid_debug.chassis_twist_risk;
            double out_x = s.y;
            double out_y = s.x;
            double out_psi = M_PI / 2.0 - s.psi;
            ofs << t << "," << out_x << "," << out_y << "," << s.z << "," << s.phi << "," << s.theta << "," << out_psi << "," << s.vx << "," << s.vy << "," << s.vz << "," << s.p << "," << s.q << "," << s.r << "," << frenet.s << "," << frenet.n << "," << frenet.d_heading << "," << track->get_curvature(frenet.s) << "," << input.steering << "," << input.throttle << "," << input.brake << "," << input.gear << "," << s.speed() << "," << target_speed_log << ",0," << lateral_error << "," << (frenet.d_heading * 180.0 / M_PI) << ",0,0,0,0," << (vehicle->is_drs_active() ? 1 : 0) << ","
                << target_speed_base_log << "," << target_speed_preview_log << "," << target_speed_final_log << ","
                << pid_debug.steering_preview_distance_used << ","
                << pid_debug.target_lateral_offset << "," << pid_debug.line_curvature_now << ","
                << pid_debug.line_curvature_ahead << "," << pid_debug.line_curvature_behind << ","
                << delta_target_log << "," << target_sw_log << "," << delta_sw_log << "," << ddelta_sw_log << ","
                << required_delta_preview_log << "," << actual_delta_tire_log << ","
                << predicted_delta_tire_log << "," << preview_delta_gap_log << ","
                << raw_delta_target_log << "," << inner_target_delta_tire_log << ","
                << actual_delta_tire_rate_pre_log << ","
                << lpv_prediction_error_log << "," << predicted_delta_tire_rate_log << ","
                << steering_prediction_horizon_log << ","
                << steering_prediction_target_time_log << ","
                << steering_prediction_valid_log << ","
                << steering_prediction_error_valid_log << ","
                << lpv_outer_sequence_log << "," << required_reference_s_log << ","
                << required_reference_time_log << "," << model_yaw_rate_map_log << ","
                << actual_yaw_rate_map_log << "," << yaw_rate_error_map_log << ","
                << sw_error_log << "," << torque_lag_lead_sw_log << ","
                << T_driver_raw_log << "," << T_driver_clamped_log << ","
                << torque_headroom_log << "," << T_SAT_total_log << "," << T_SAT_reflected_log << ","
                << T_SAT_feedforward_log << ","
                << pid_debug.time_to_reach_delta << ","
                << steering_saturation_log << "," << torque_speed_multiplier_log << ","
                << pid_debug.torque_speed_recovery_ratio << ","
                << steering_lag_ratio_log << "," << chassis_twist_risk_log << ","
                << effective_max_tire_steer_rate_log << ","
                << steering_actuator_speed_cap_log << "," << pid_debug.steering_reach_speed_cap << ","
                << track_margin_speed_cap_log << ","
                << pid_debug.track_margin_effective_margin << "," << pid_debug.track_margin_outward_rate << ","
                << mpc_rollout_progress_log << "," << mpc_rollout_progress_target_log << ","
                << mpc_rollout_progress_deficit_log << "," << mpc_rollout_cost_log << ","
                << mpc_selected_plan_index_log << ","
                << mpc_predicted_terminal_n_log << ","
                << mpc_predicted_terminal_heading_log << ","
                << mpc_predicted_min_margin_log << ","
                << mpc_predicted_terminal_speed_log << ","
                << mpc_predicted_signed_progress_log << ","
                << attack_selected_primitive_log << ","
                << attack_selected_primitive_kind_log << ","
                << attack_selected_speed_scale_log << ","
                << attack_selected_target_speed_log << ","
                << attack_selected_ax_log << ","
                << course_v_ref_log << ","
                << course_n_ref_log << ","
                << course_kappa_ref_log << ","
                << course_speed_error_log << ","
                << course_lateral_error_log << ","
                << course_a_ref_log << ","
                << course_target_gap_log << ","
                << dynamic_cap_gap_log << ","
                << braking_corridor_active_log << ","
                << braking_corridor_v_lower_log << ","
                << braking_corridor_v_upper_log << ","
                << attack_residual_delta_v_ref_log << ","
                << attack_residual_delta_steer_ff_log << ","
                << attack_residual_delta_ax_bias_log << ","
                << attack_residual_delta_heading_relax_log << ","
                << attack_residual_safety_gate_active_log << ","
                << attack_raw_heading_error_log << ","
                << attack_effective_heading_error_log << ","
                << attack_ax_before_residual_log << ","
                << attack_ax_after_residual_log << ","
                << longitudinal_mode_log << ","
                << healthy_underspeed_log << ","
                << accel_headroom_log << ","
                << track_phase_log << ","
                << phase_speed_bias_log << ","
                << phase_margin_bias_log << ","
                << phase_progress_bias_log << ","
                << attack_segment_id_log << ","
                << attack_allowed_log << ","
                << attack_static_speed_cap_log << ","
                << attack_dynamic_speed_cap_log << ","
                << attack_dynamic_base_speed_log << ","
                << attack_curvature_speed_cap_log << ","
                << attack_steering_speed_cap_log << ","
                << attack_margin_speed_cap_log << ","
                << attack_heading_speed_cap_log << ","
                << attack_outward_speed_cap_log << ","
                << attack_limiting_cap_reason_log << ","
                << attack_rollout_candidate_count_log << ","
                << attack_rollout_reject_count_log << ","
                << attack_rollout_boundary_reject_count_log << ","
                << attack_rollout_nonfinite_reject_count_log << ","
                << attack_all_rejected_log << ","
                << attack_cost_tracking_log << ","
                << attack_cost_speed_log << ","
                << attack_cost_boundary_log << ","
                << attack_cost_control_smoothness_log << ","
                << attack_cost_progress_log << ","
                << attack_cost_primitive_bias_log << ","
                << attack_cost_overspeed_log << ","
                << attack_cost_underspeed_log << ","
                << attack_cost_time_progress_log << ","
                << attack_best_tracking_cost_log << ","
                << attack_best_boundary_recovery_cost_log << ","
                << attack_best_low_speed_pure_pursuit_cost_log << ","
                << attack_best_speed_scale_065_cost_log << ","
                << attack_best_speed_scale_080_cost_log << ","
                << attack_best_speed_scale_095_cost_log << ","
                << attack_best_speed_scale_105_cost_log << ","
                << attack_best_speed_scale_095_cost_delta_log << ","
                << attack_best_speed_scale_095_tracking_delta_log << ","
                << attack_best_speed_scale_095_speed_delta_log << ","
                << attack_best_speed_scale_095_boundary_delta_log << ","
                << attack_best_speed_scale_095_control_smoothness_delta_log << ","
                << attack_best_speed_scale_095_progress_delta_log << ","
                << attack_best_speed_scale_095_primitive_bias_delta_log << ","
                << pid_debug.corner_throttle_steer_ratio << ","
                << pid_debug.corner_throttle_lateral_utilization << ","
                << pid_debug.corner_throttle_lateral_ratio << ","
                << pid_debug.corner_throttle_drive_slip << ","
                << pid_debug.corner_throttle_drive_slip_ratio << ","
                << pid_debug.corner_throttle_limit_ratio << ","
                << pid_debug.corner_throttle_scale << ","
                << phi_twist_log << "," << dphi_twist_log << ","
                << pid_debug.stability_vy_penalty << "," << pid_debug.stability_yaw_penalty << ","
                << pid_debug.stability_slip_penalty << "," << pid_debug.stability_penalty << ","
                << pid_debug.estimated_front_slip << std::endl;
            next_output_log_time += config.output_rate;
        }
    }

    finalize_result_metrics();

    if (!result.lap_completed) {
        std::ostringstream oss;
        oss << "Lap was not completed within simulation_time=" << sim_time
            << " s; distance=" << result.distance_traveled << " m.";
        fail_result(result, "lap_not_completed", oss.str(), sim_time);
    }

    // MPC ソルバー統計を結果に記録
    if ((use_lpv_lqr_driver && lpv_lqr_driver) ||
        (use_mpc_driver && mpc_driver) ||
        (use_mpc_v2_driver && mpc_v2_driver) ||
        (use_attack_driver && attack_driver)) {
        const auto& mpc_stats = use_lpv_lqr_driver
            ? lpv_lqr_driver->stats()
            : (use_attack_driver
                ? attack_driver->stats()
                : (use_mpc_v2_driver ? mpc_v2_driver->stats() : mpc_driver->stats()));
        result.mpc_solve_count       = mpc_stats.solve_count;
        result.mpc_avg_solve_time_ms = mpc_stats.avg_solve_time_ms();
        result.mpc_max_solve_time_ms = mpc_stats.max_solve_time_ms;
        result.mpc_fail_count        = mpc_stats.fail_count;
        if (use_lpv_lqr_driver) {
            result.actuator_prediction_count = mpc_stats.actuator_prediction_count;
            result.actuator_prediction_bias = mpc_stats.actuator_prediction_bias();
            result.actuator_prediction_mae = mpc_stats.actuator_prediction_mae();
            result.actuator_prediction_rmse = mpc_stats.actuator_prediction_rmse();
            result.actuator_prediction_max_abs_error =
                mpc_stats.actuator_prediction_max_abs_error;
        }
    }

    return result;
}

int exit_code_for_result(const LapResult& result) {
    if (result.status != "failed") {
        return 0;
    }
    if (result.failure_code == "autocross_csv_load_failed" ||
        result.failure_code == "track_csv_load_failed") {
        return 2;
    }
    if (result.failure_code == "physics_divergence") {
        return 3;
    }
    if (result.failure_code == "early_stall") {
        return 4;
    }
    if (result.failure_code == "tire_model_load_failed") {
        return 5;
    }
    return 1;
}

static void warn_deprecated_margin(const std::string& arg) {
    std::cerr << "Warning: " << arg << " is deprecated (Issue #26); use --friction-margin "
              << "(single margin applied once to the lateral/braking limits)." << std::endl;
}

int main(int argc, char** argv) {
    SimulationConfig config;
    // Argument parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--track" && i + 1 < argc) config.track_name = argv[++i];
        else if (arg == "--track-closed") config.track_closed = true;
        else if (arg == "--track-open") config.track_closed = false;
        else if (arg == "--gain-preset" && i + 1 < argc) config.gain_preset = argv[++i];
        else if (arg == "--radius" && i + 1 < argc) config.track_radius = std::stod(argv[++i]);
        else if ((arg == "--track-width" || arg == "--width") && i + 1 < argc) config.track_width = std::stod(argv[++i]);
        else if (arg == "--time" && i + 1 < argc) { config.simulation_time = std::stod(argv[++i]); config.simulation_time_explicit = true; }
        else if (arg == "--dt" && i + 1 < argc) { config.dt = std::stod(argv[++i]); config.dt_explicit = true; }
        else if (arg == "--output-rate" && i + 1 < argc) config.output_rate = std::stod(argv[++i]);
        else if (arg == "--output" && i + 1 < argc) config.output_file = argv[++i];
        else if (arg == "--rl-teacher-output" && i + 1 < argc) config.rl_teacher_output_file = argv[++i];
        else if ((arg == "--velocity-profile-in" || arg == "--velocity-profile-input") && i + 1 < argc) config.velocity_profile_input_file = argv[++i];
        else if ((arg == "--velocity-profile-out" || arg == "--velocity-profile-output") && i + 1 < argc) config.velocity_profile_output_file = argv[++i];
        else if (arg == "--params_file" && i + 1 < argc) config.params_file = argv[++i];
        else if (arg == "--params-json" && i + 1 < argc) config.params_json_str = argv[++i];
        else if (arg == "--driver" && i + 1 < argc) config.driver_mode = argv[++i];
        else if (arg == "--eval-mode" && i + 1 < argc) config.eval_mode = argv[++i];
        else if (arg == "--rl-actor-dir" && i + 1 < argc) config.rl_actor_dir = argv[++i];
        else if (arg == "--rl-steer-torque-rate-limit" && i + 1 < argc) config.rl_steer_torque_rate_limit = std::stod(argv[++i]);
        else if (arg == "--rl-steering-slew-rate" && i + 1 < argc) config.rl_steer_torque_rate_limit = std::stod(argv[++i]);
        else if (arg == "--mpc-horizon" && i + 1 < argc) config.mpc_horizon = std::stoi(argv[++i]);
        else if (arg == "--mpc-dt" && i + 1 < argc) config.mpc_dt = std::stod(argv[++i]);
        else if (arg == "--lpv-model" && i + 1 < argc) config.lpv_model = argv[++i];
        else if (arg == "--lpv-lateral-weight-scale" && i + 1 < argc) config.lpv_lateral_weight_scale = std::stod(argv[++i]);
        else if (arg == "--lpv-heading-weight-scale" && i + 1 < argc) config.lpv_heading_weight_scale = std::stod(argv[++i]);
        else if (arg == "--attack-speed-lateral-safety" && i + 1 < argc) { config.attack_speed_lateral_safety = std::stod(argv[++i]); warn_deprecated_margin(arg); }
        else if (arg == "--attack-speed-braking-safety" && i + 1 < argc) { config.attack_speed_braking_safety = std::stod(argv[++i]); warn_deprecated_margin(arg); }
        else if (arg == "--friction-margin" && i + 1 < argc) config.friction_margin = std::stod(argv[++i]);
        else if (arg == "--attack-speed-accel-limit" && i + 1 < argc) config.attack_speed_accel_limit = std::stod(argv[++i]);
        else if (arg == "--attack-rollout-understeer-gradient" && i + 1 < argc) config.attack_rollout_understeer_gradient = std::stod(argv[++i]);
        else if (arg == "--attack-floor-straight" && i + 1 < argc) config.attack_floor_straight = std::stod(argv[++i]);
        else if (arg == "--attack-floor-exit" && i + 1 < argc) config.attack_floor_exit = std::stod(argv[++i]);
        else if (arg == "--attack-floor-complex" && i + 1 < argc) config.attack_floor_complex = std::stod(argv[++i]);
        else if (arg == "--attack-floor-turn-in" && i + 1 < argc) config.attack_floor_turn_in = std::stod(argv[++i]);
        else if (arg == "--attack-floor-apex" && i + 1 < argc) config.attack_floor_apex = std::stod(argv[++i]);
        else if (arg == "--attack-floor-brake" && i + 1 < argc) config.attack_floor_brake = std::stod(argv[++i]);
        else if (arg == "--attack-braking-corridor-decel-scale" && i + 1 < argc) config.attack_braking_corridor_decel_scale = std::stod(argv[++i]);
        else if (arg == "--attack-braking-corridor-band-scale" && i + 1 < argc) config.attack_braking_corridor_band_scale = std::stod(argv[++i]);
        else if (arg == "--attack-braking-corridor-mild-accel-scale" && i + 1 < argc) config.attack_braking_corridor_mild_accel_scale = std::stod(argv[++i]);
        else if (arg == "--attack-braking-corridor-overspeed-brake-gain" && i + 1 < argc) config.attack_braking_corridor_overspeed_brake_gain = std::stod(argv[++i]);
        else if (arg == "--attack-braking-corridor-guard-only") config.attack_braking_corridor_guard_only = true;
        else if (arg == "--attack-braking-corridor-band-target") config.attack_braking_corridor_guard_only = false;
        else if (arg == "--attack-w-y" && i + 1 < argc) config.attack_w_y = std::stod(argv[++i]);
        else if (arg == "--attack-w-psi" && i + 1 < argc) config.attack_w_psi = std::stod(argv[++i]);
        else if (arg == "--attack-w-vx" && i + 1 < argc) config.attack_w_vx = std::stod(argv[++i]);
        else if (arg == "--attack-w-delta" && i + 1 < argc) config.attack_w_delta = std::stod(argv[++i]);
        else if (arg == "--attack-w-ax" && i + 1 < argc) config.attack_w_ax = std::stod(argv[++i]);
        else if (arg == "--attack-w-delta-rate" && i + 1 < argc) config.attack_w_delta_rate = std::stod(argv[++i]);
        else if (arg == "--attack-w-ax-rate" && i + 1 < argc) config.attack_w_ax_rate = std::stod(argv[++i]);
        else if (arg == "--attack-terminal-progress-weight" && i + 1 < argc) config.attack_terminal_progress_weight = std::stod(argv[++i]);
        else if (arg == "--attack-progress-target-fraction" && i + 1 < argc) config.attack_progress_target_fraction = std::stod(argv[++i]);
        else if (arg == "--attack-soft-boundary-weight" && i + 1 < argc) config.attack_soft_boundary_weight = std::stod(argv[++i]);
        else if (arg == "--attack-outward-rate-weight" && i + 1 < argc) config.attack_outward_rate_weight = std::stod(argv[++i]);
        else if (arg == "--attack-speed-cap-file" && i + 1 < argc) config.attack_speed_cap_file = argv[++i];
        else if (arg == "--attack-residual-profile" && i + 1 < argc) config.attack_residual_profile_file = argv[++i];
        else if (arg == "--driver-capability-profile" && i + 1 < argc) config.driver_capability_profile_file = argv[++i];
        else if (arg == "--laps" && i + 1 < argc) config.laps = std::stoi(argv[++i]);
        else if (arg == "--output_json") config.output_json_result = true;
        else if (arg == "--allow-defaults") config.allow_defaults = true;
        else if (arg == "--initial-speed" && i + 1 < argc) { config.initial_speed = std::stod(argv[++i]); config.initial_speed_explicit = true; }
        else if (arg == "--max-speed" && i + 1 < argc) config.max_speed = std::stod(argv[++i]);
        else if (arg == "--max-lat-accel" && i + 1 < argc) {
            config.max_lat_accel = std::stod(argv[++i]);
            config.max_lat_accel_explicit = true;
        }
        else if (arg == "--max-lon-accel" && i + 1 < argc) { config.max_lon_accel = std::stod(argv[++i]); config.max_lon_explicit = true; }
        else if (arg == "--max-lon-decel" && i + 1 < argc) { config.max_lon_decel = std::stod(argv[++i]); config.max_lon_explicit = true; }
        else if (arg == "--ggv" && i + 1 < argc) config.ggv_file = argv[++i];
        else if (arg == "--wind" && i + 1 < argc) {
            const std::string w = argv[++i];
            const auto comma = w.find(',');
            if (comma == std::string::npos) { std::cerr << "--wind expects wx,wy [m/s]" << std::endl; return 1; }
            config.wind_x = std::stod(w.substr(0, comma));
            config.wind_y = std::stod(w.substr(comma + 1));
        }
        else if (arg == "--kp-steer" && i + 1 < argc) { config.Kp_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--kd-steer" && i + 1 < argc) { config.Kd_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--ki-steer" && i + 1 < argc) { config.Ki_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--kh-steer" && i + 1 < argc) { config.Kh_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--kff-steer" && i + 1 < argc) { config.Kff_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--kp-speed" && i + 1 < argc) { config.Kp_speed = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--ki-speed" && i + 1 < argc) { config.Ki_speed = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--max-throttle" && i + 1 < argc) { config.max_throttle = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--max-steer" && i + 1 < argc) { config.max_steer = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--brake-gain-ratio" && i + 1 < argc) { config.brake_gain_ratio = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--corner-throttle-reduction" && i + 1 < argc) { config.corner_throttle_reduction = std::stod(argv[++i]); config.use_custom_gains = true; }
        else if (arg == "--corner-throttle-torque-reference" && i + 1 < argc) config.corner_throttle_torque_reference = std::stod(argv[++i]);
        else if (arg == "--corner-throttle-lateral-utilization-start" && i + 1 < argc) config.corner_throttle_lateral_utilization_start = std::stod(argv[++i]);
        else if (arg == "--corner-throttle-lateral-utilization-full" && i + 1 < argc) config.corner_throttle_lateral_utilization_full = std::stod(argv[++i]);
        else if (arg == "--corner-throttle-drive-slip-start" && i + 1 < argc) config.corner_throttle_drive_slip_start = std::stod(argv[++i]);
        else if (arg == "--corner-throttle-drive-slip-full" && i + 1 < argc) config.corner_throttle_drive_slip_full = std::stod(argv[++i]);
        else if (arg == "--target-speed-preview-distance" && i + 1 < argc) config.target_speed_preview_distance = std::stod(argv[++i]);
        else if (arg == "--torque-speed-scale" && i + 1 < argc) config.torque_speed_scale = std::stod(argv[++i]);
        else if (arg == "--torque-preview-distance" && i + 1 < argc) config.torque_preview_distance = std::stod(argv[++i]);
        else if (arg == "--torque-outer-loop-scale" && i + 1 < argc) config.torque_outer_loop_scale = std::stod(argv[++i]);
        else if ((arg == "--driver-torque-max" || arg == "--steering-driver-torque-max" || arg == "--steering-torque-max") && i + 1 < argc) {
            config.steering_driver_torque_max = std::stod(argv[++i]);
        }
        else if ((arg == "--driver-torque-rate-max" || arg == "--steering-driver-torque-rate-max" || arg == "--steering-torque-rate-max") && i + 1 < argc) {
            config.steering_driver_torque_rate_max = std::stod(argv[++i]);
        }
        else if (arg == "--max-tire-steer-rate" && i + 1 < argc) config.max_tire_steer_rate = std::stod(argv[++i]);
        else if (arg == "--torque-steer-rate-reference-torque" && i + 1 < argc) config.torque_steer_rate_reference_torque = std::stod(argv[++i]);
        else if (arg == "--torque-steer-rate-reference-inertia" && i + 1 < argc) config.torque_steer_rate_reference_inertia = std::stod(argv[++i]);
        else if (arg == "--torque-steer-rate-authority-exponent" && i + 1 < argc) config.torque_steer_rate_authority_exponent = std::stod(argv[++i]);
        else if (arg == "--torque-steer-rate-min-scale" && i + 1 < argc) config.torque_steer_rate_min_scale = std::stod(argv[++i]);
        else if (arg == "--torque-steer-rate-max-scale" && i + 1 < argc) config.torque_steer_rate_max_scale = std::stod(argv[++i]);
        else if (arg == "--torque-lag-lead-gain" && i + 1 < argc) config.torque_lag_lead_gain = std::stod(argv[++i]);
        else if (arg == "--torque-lag-lead-time" && i + 1 < argc) config.torque_lag_lead_time = std::stod(argv[++i]);
        else if (arg == "--torque-lag-lead-max-tire-angle" && i + 1 < argc) config.torque_lag_lead_max_tire_angle = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-gain" && i + 1 < argc) config.torque_speed_recovery_gain = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-curvature" && i + 1 < argc) config.torque_speed_recovery_curvature = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-lag-ratio" && i + 1 < argc) config.torque_speed_recovery_lag_ratio = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-lateral-error" && i + 1 < argc) config.torque_speed_recovery_lateral_error = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-heading-error" && i + 1 < argc) config.torque_speed_recovery_heading_error = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-preview-gap" && i + 1 < argc) config.torque_speed_recovery_preview_gap = std::stod(argv[++i]);
        else if (arg == "--torque-speed-recovery-reach-time" && i + 1 < argc) config.torque_speed_recovery_reach_time = std::stod(argv[++i]);
        else if (arg == "--torque-reach-speed-cap" && i + 1 < argc) config.torque_reach_speed_cap = std::stod(argv[++i]);
        else if (arg == "--torque-reach-time-start" && i + 1 < argc) config.torque_reach_time_start = std::stod(argv[++i]);
        else if (arg == "--torque-reach-time-full" && i + 1 < argc) config.torque_reach_time_full = std::stod(argv[++i]);
        else if (arg == "--torque-reach-outward-rate-start" && i + 1 < argc) config.torque_reach_outward_rate_start = std::stod(argv[++i]);
        else if (arg == "--torque-reach-outward-rate-full" && i + 1 < argc) config.torque_reach_outward_rate_full = std::stod(argv[++i]);
        else if (arg == "--torque-reach-min-preview-gap" && i + 1 < argc) config.torque_reach_min_preview_gap = std::stod(argv[++i]);
        else if (arg == "--track-margin-speed-cap" && i + 1 < argc) config.track_margin_speed_cap = std::stod(argv[++i]);
        else if (arg == "--track-margin-speed-release" && i + 1 < argc) config.track_margin_speed_release = std::stod(argv[++i]);
        else if (arg == "--track-margin-preview-time" && i + 1 < argc) config.track_margin_preview_time = std::stod(argv[++i]);
        else if (arg == "--steer-inner-ki" && i + 1 < argc) config.steer_inner_ki = std::stod(argv[++i]);
        else if (arg == "--steer-inner-omega" && i + 1 < argc) config.steer_inner_omega = std::stod(argv[++i]);
        else if (arg == "--steer-inner-zeta" && i + 1 < argc) config.steer_inner_zeta = std::stod(argv[++i]);
        else if (arg == "--steering-preview-distance" && i + 1 < argc) config.steering_preview_distance = std::stod(argv[++i]);
        else if (arg == "--steering-preview-time" && i + 1 < argc) config.steering_preview_time = std::stod(argv[++i]);
        else if (arg == "--steering-preview-min-distance" && i + 1 < argc) config.steering_preview_min_distance = std::stod(argv[++i]);
        else if (arg == "--steering-preview-max-distance" && i + 1 < argc) config.steering_preview_max_distance = std::stod(argv[++i]);
        else if (arg == "--kpp-steer" && i + 1 < argc) config.kpp_steer = std::stod(argv[++i]);
        else if (arg == "--understeer-gradient" && i + 1 < argc) config.understeer_gradient = std::stod(argv[++i]);
        else if (arg == "--torque-speed-cap-preview-time" && i + 1 < argc) config.torque_speed_cap_preview_time = std::stod(argv[++i]);
        else if (arg == "--torque-speed-cap-preview-distance" && i + 1 < argc) config.torque_speed_cap_preview_distance = std::stod(argv[++i]);
        else if (arg == "--torque-speed-cap-min-preview-distance" && i + 1 < argc) config.torque_speed_cap_min_preview_distance = std::stod(argv[++i]);
        else if (arg == "--torque-speed-cap-max-preview-distance" && i + 1 < argc) config.torque_speed_cap_max_preview_distance = std::stod(argv[++i]);
        else if (arg == "--line-planner") config.line_planner_enabled = true;
        else if (arg == "--no-line-planner") config.line_planner_disabled = true;
        else if (arg == "--line-offset-fraction" && i + 1 < argc) config.line_offset_fraction = std::stod(argv[++i]);
        else if (arg == "--line-safety-margin" && i + 1 < argc) config.line_safety_margin = std::stod(argv[++i]);
        else if (arg == "--line-preview-distance" && i + 1 < argc) config.line_preview_distance = std::stod(argv[++i]);
        else if (arg == "--line-smoothing-distance" && i + 1 < argc) config.line_smoothing_distance = std::stod(argv[++i]);
        else if (arg == "--line-scan-step" && i + 1 < argc) config.line_scan_step = std::stod(argv[++i]);
        else if (arg == "--line-entry-offset-scale" && i + 1 < argc) config.line_entry_offset_scale = std::stod(argv[++i]);
        else if (arg == "--line-apex-offset-scale" && i + 1 < argc) config.line_apex_offset_scale = std::stod(argv[++i]);
        else if (arg == "--line-exit-offset-scale" && i + 1 < argc) config.line_exit_offset_scale = std::stod(argv[++i]);
        else if (arg == "--line-track-geometry") config.line_track_geometry_enabled = true;
        else if (arg == "--no-line-track-geometry") config.line_track_geometry_disabled = true;
        else if (arg == "--no-line-profile-curvature") config.line_profile_curvature = false;
        else if (arg == "--torque-stability-vy-threshold" && i + 1 < argc) config.torque_stability_vy_threshold = std::stod(argv[++i]);
        else if (arg == "--torque-stability-vy-gain" && i + 1 < argc) config.torque_stability_vy_gain = std::stod(argv[++i]);
        else if (arg == "--torque-stability-yaw-threshold" && i + 1 < argc) config.torque_stability_yaw_threshold = std::stod(argv[++i]);
        else if (arg == "--torque-stability-yaw-gain" && i + 1 < argc) config.torque_stability_yaw_gain = std::stod(argv[++i]);
        else if (arg == "--torque-stability-slip-threshold" && i + 1 < argc) config.torque_stability_slip_threshold = std::stod(argv[++i]);
        else if (arg == "--torque-stability-slip-gain" && i + 1 < argc) config.torque_stability_slip_gain = std::stod(argv[++i]);
        else if (arg == "--distance-cg-front" && i + 1 < argc) config.distance_cg_front = std::stod(argv[++i]);
    }

    if (
        config.gain_preset != "default" &&
        config.gain_preset != "conservative" &&
        config.gain_preset != "aggressive" &&
        config.gain_preset != "optimized_figure_eight" &&
        config.gain_preset != "optimized_skidpad" &&
        config.gain_preset != "optimized_autocross" &&
        config.gain_preset != "optimized_autocross_fast"
    ) {
        std::cerr << "Unknown gain preset: " << config.gain_preset << std::endl;
        return 1;
    }

    if (config.driver_mode != "pid" &&
        config.driver_mode != "lpv_lqr" &&
        config.driver_mode != "mpc" &&
        config.driver_mode != "mpc_v2" &&
        config.driver_mode != "attack" &&
        config.driver_mode != "rl") {
        std::cerr << "Unknown driver mode: " << config.driver_mode << std::endl;
        return 1;
    }
    if (config.lpv_model != "bicycle4" && config.lpv_model != "steering6") {
        std::cerr << "Unknown LPV model: " << config.lpv_model << std::endl;
        return 1;
    }
    if (config.lpv_lateral_weight_scale <= 0.0 ||
        !std::isfinite(config.lpv_lateral_weight_scale) ||
        config.lpv_heading_weight_scale <= 0.0 ||
        !std::isfinite(config.lpv_heading_weight_scale)) {
        std::cerr << "LPV weight scales must be finite and positive." << std::endl;
        return 1;
    }
    if (config.eval_mode != "full" && config.eval_mode != "fast") {
        std::cerr << "Unknown eval mode: " << config.eval_mode << std::endl;
        return 1;
    }
    if (config.driver_mode == "rl" && config.rl_actor_dir.empty()) {
        std::cerr << "--rl-actor-dir is required when --driver rl is specified." << std::endl;
        return 1;
    }
    if (config.dt <= 0.0 || !std::isfinite(config.dt)) {
        std::cerr << "--dt must be finite and positive." << std::endl;
        return 1;
    }
    if (config.output_rate < config.dt || !std::isfinite(config.output_rate)) {
        std::cerr << "--output-rate must be finite and at least --dt." << std::endl;
        return 1;
    }
    if (config.max_speed <= 0.0 || !std::isfinite(config.max_speed)) {
        std::cerr << "--max-speed must be finite and positive." << std::endl;
        return 1;
    }
    if (config.laps < 1) {
        std::cerr << "--laps must be >= 1." << std::endl;
        return 1;
    }
    if (!(config.friction_margin > 0.0 && config.friction_margin <= 1.0)) {
        std::cerr << "--friction-margin must be in (0, 1]." << std::endl;
        return 1;
    }
    if (!config.velocity_profile_input_file.empty() &&
        !config.velocity_profile_output_file.empty()) {
        std::cerr << "--velocity-profile-in and --velocity-profile-out are mutually exclusive." << std::endl;
        return 1;
    }

    LapResult result;
    try {
        result = run_laptime_simulation(config);
    } catch (const DiagnosticFailure& e) {
        result.status = "failed";
        result.failure_code = e.code;
        result.failure_message = e.what();
    } catch (const std::exception& e) {
        result.status = "failed";
        result.failure_code = "unhandled_exception";
        result.failure_message = e.what();
    }

    if (config.output_json_result) {
        // NaN/Inf ガード: JSON は nan/inf を受け付けないため 0 に置換
        auto safe_json = [](double v) { return (std::isnan(v) || std::isinf(v)) ? 0.0 : v; };
        std::cout << "%%JSON_BEGIN%%" << std::endl;
        std::cout << "{"
                  << "\"status\":\"" << escape_json_string(result.status) << "\""
                  << ",\"failure_code\":\"" << escape_json_string(result.failure_code) << "\""
                  << ",\"failure_message\":\"" << escape_json_string(result.failure_message) << "\""
                  << ",\"degraded_mode\":" << (result.degraded_mode ? "true" : "false")
                  << ",\"degraded_reason\":\"" << escape_json_string(result.degraded_reason) << "\""
                  << ",\"track_source\":\"" << escape_json_string(result.track_source) << "\""
                  << ",\"track_csv_path\":\"" << escape_json_string(result.track_csv_path) << "\""
                  << ",\"artifact_defaults\":\"" << escape_json_string(result.artifact_defaults) << "\""
                  << ",\"tire_model_type\":\"" << escape_json_string(result.tire_model_type) << "\""
                  << ",\"tire_model_name\":\"" << escape_json_string(result.tire_model_name) << "\""
                  << ",\"tire_runtime\":\"" << escape_json_string(result.tire_runtime) << "\""
                  << ",\"failure_time\":" << safe_json(result.failure_time)
                  << ",\"lap_time\":" << safe_json(result.lap_time)
                  << ",\"lap_completed\":" << (result.lap_completed ? "true" : "false")
                  << ",\"distance_traveled\":" << safe_json(result.distance_traveled)
                  << ",\"max_lateral_error\":" << safe_json(result.max_lateral_error)
                  << ",\"avg_lateral_error\":" << safe_json(result.avg_lateral_error)
                  << ",\"max_speed\":" << safe_json(result.max_speed)
                  << ",\"avg_speed\":" << safe_json(result.avg_speed)
                  << ",\"rms_heading_error\":" << safe_json(result.rms_heading_error)
                  << ",\"steering_smoothness\":" << safe_json(result.steering_smoothness)
                  << ",\"throttle_smoothness\":" << safe_json(result.throttle_smoothness)
                  << ",\"brake_smoothness\":" << safe_json(result.brake_smoothness)
                  << ",\"steering_saturation_ratio\":" << safe_json(result.steering_saturation_ratio)
                  << ",\"mean_abs_sw_error\":" << safe_json(result.mean_abs_sw_error)
                  << ",\"max_abs_sw_error\":" << safe_json(result.max_abs_sw_error)
	                  << ",\"mean_steering_lag_ratio\":" << safe_json(result.mean_steering_lag_ratio)
	                  << ",\"max_steering_lag_ratio\":" << safe_json(result.max_steering_lag_ratio)
	                  << ",\"avg_torque_speed_multiplier\":" << safe_json(result.avg_torque_speed_multiplier)
	                  << ",\"mean_torque_speed_loss\":" << safe_json(result.mean_torque_speed_loss)
	                  << ",\"mean_steering_cap_speed_loss\":" << safe_json(result.mean_steering_cap_speed_loss)
	                  << ",\"mean_steering_reach_speed_loss\":" << safe_json(result.mean_steering_reach_speed_loss)
	                  << ",\"mean_track_margin_speed_loss\":" << safe_json(result.mean_track_margin_speed_loss)
	                  << ",\"mean_corner_throttle_loss\":" << safe_json(result.mean_corner_throttle_loss)
	                  << ",\"mean_chassis_twist_risk\":" << safe_json(result.mean_chassis_twist_risk)
	                  << ",\"chassis_twist_risk_integral\":" << safe_json(result.chassis_twist_risk_integral)
	                  << ",\"control_intervention_cost\":" << safe_json(result.control_intervention_cost)
	                  << ",\"vehicle_health_penalty\":" << safe_json(result.vehicle_health_penalty)
	                  << ",\"driver_masking_risk\":" << safe_json(result.driver_masking_risk)
	                  << ",\"driver_intervention_cost\":" << safe_json(result.driver_intervention_cost)
	                  << ",\"mean_mpc_rollout_progress_deficit\":" << safe_json(result.mean_mpc_rollout_progress_deficit)
	                  << ",\"max_mpc_rollout_progress_deficit\":" << safe_json(result.max_mpc_rollout_progress_deficit)
                      << ",\"mean_mpc_predicted_min_margin\":" << safe_json(result.mean_mpc_predicted_min_margin)
                      << ",\"min_mpc_predicted_min_margin\":" << safe_json(result.min_mpc_predicted_min_margin)
                      << ",\"mean_mpc_predicted_terminal_abs_n\":" << safe_json(result.mean_mpc_predicted_terminal_abs_n)
                      << ",\"mean_mpc_predicted_terminal_abs_heading\":" << safe_json(result.mean_mpc_predicted_terminal_abs_heading)
                      << ",\"mean_mpc_predicted_terminal_speed\":" << safe_json(result.mean_mpc_predicted_terminal_speed)
                      << ",\"mean_mpc_predicted_signed_progress\":" << safe_json(result.mean_mpc_predicted_signed_progress)
                      << ",\"last_mpc_selected_plan_index\":" << result.last_mpc_selected_plan_index
                      << ",\"mean_attack_cost_tracking\":" << safe_json(result.mean_attack_cost_tracking)
                      << ",\"mean_attack_cost_speed\":" << safe_json(result.mean_attack_cost_speed)
                      << ",\"mean_attack_cost_boundary\":" << safe_json(result.mean_attack_cost_boundary)
                      << ",\"mean_attack_cost_control_smoothness\":" << safe_json(result.mean_attack_cost_control_smoothness)
                      << ",\"mean_attack_cost_progress\":" << safe_json(result.mean_attack_cost_progress)
                      << ",\"mean_attack_cost_primitive_bias\":" << safe_json(result.mean_attack_cost_primitive_bias)
                      << ",\"mean_attack_cost_overspeed\":" << safe_json(result.mean_attack_cost_overspeed)
                      << ",\"mean_attack_cost_underspeed\":" << safe_json(result.mean_attack_cost_underspeed)
                      << ",\"mean_attack_cost_time_progress\":" << safe_json(result.mean_attack_cost_time_progress)
                      << ",\"mean_attack_rollout_reject_count\":" << safe_json(result.mean_attack_rollout_reject_count)
                      << ",\"max_attack_rollout_reject_count\":" << result.max_attack_rollout_reject_count
                      << ",\"mean_attack_rollout_boundary_reject_count\":" << safe_json(result.mean_attack_rollout_boundary_reject_count)
                      << ",\"mean_attack_rollout_nonfinite_reject_count\":" << safe_json(result.mean_attack_rollout_nonfinite_reject_count)
	                      << ",\"mean_attack_best_tracking_cost\":" << safe_json(result.mean_attack_best_tracking_cost)
	                      << ",\"mean_attack_best_boundary_recovery_cost\":" << safe_json(result.mean_attack_best_boundary_recovery_cost)
	                      << ",\"mean_attack_best_low_speed_pure_pursuit_cost\":" << safe_json(result.mean_attack_best_low_speed_pure_pursuit_cost)
	                      << ",\"mean_attack_best_speed_scale_065_cost\":" << safe_json(result.mean_attack_best_speed_scale_065_cost)
	                      << ",\"mean_attack_best_speed_scale_080_cost\":" << safe_json(result.mean_attack_best_speed_scale_080_cost)
	                      << ",\"mean_attack_best_speed_scale_095_cost\":" << safe_json(result.mean_attack_best_speed_scale_095_cost)
	                      << ",\"mean_attack_best_speed_scale_105_cost\":" << safe_json(result.mean_attack_best_speed_scale_105_cost)
	                      << ",\"mean_attack_best_speed_scale_095_cost_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_cost_delta)
	                      << ",\"mean_attack_best_speed_scale_095_tracking_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_tracking_delta)
	                      << ",\"mean_attack_best_speed_scale_095_speed_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_speed_delta)
	                      << ",\"mean_attack_best_speed_scale_095_boundary_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_boundary_delta)
	                      << ",\"mean_attack_best_speed_scale_095_control_smoothness_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_control_smoothness_delta)
	                      << ",\"mean_attack_best_speed_scale_095_progress_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_progress_delta)
	                      << ",\"mean_attack_best_speed_scale_095_primitive_bias_delta\":" << safe_json(result.mean_attack_best_speed_scale_095_primitive_bias_delta)
	                      << ",\"last_attack_selected_primitive\":" << result.last_attack_selected_primitive
	                      << ",\"last_attack_selected_primitive_kind\":\"" << escape_json_string(result.last_attack_selected_primitive_kind) << "\""
	                      << ",\"last_attack_selected_speed_scale\":" << safe_json(result.last_attack_selected_speed_scale)
	                      << ",\"last_attack_selected_target_speed\":" << safe_json(result.last_attack_selected_target_speed)
	                      << ",\"last_attack_selected_ax\":" << safe_json(result.last_attack_selected_ax)
	                      << ",\"last_course_v_ref\":" << safe_json(result.last_course_v_ref)
	                      << ",\"last_course_n_ref\":" << safe_json(result.last_course_n_ref)
	                      << ",\"last_course_kappa_ref\":" << safe_json(result.last_course_kappa_ref)
	                      << ",\"last_course_a_ref\":" << safe_json(result.last_course_a_ref)
	                      << ",\"mean_course_speed_error\":" << safe_json(result.mean_course_speed_error)
	                      << ",\"mean_abs_course_lateral_error\":" << safe_json(result.mean_abs_course_lateral_error)
	                      << ",\"mean_course_target_gap\":" << safe_json(result.mean_course_target_gap)
	                      << ",\"mean_dynamic_cap_gap\":" << safe_json(result.mean_dynamic_cap_gap)
	                      << ",\"mean_attack_residual_delta_v_ref\":" << safe_json(result.mean_attack_residual_delta_v_ref)
	                      << ",\"mean_attack_residual_delta_steer_ff\":" << safe_json(result.mean_attack_residual_delta_steer_ff)
	                      << ",\"mean_attack_residual_delta_ax_bias\":" << safe_json(result.mean_attack_residual_delta_ax_bias)
	                      << ",\"mean_attack_residual_delta_heading_relax\":" << safe_json(result.mean_attack_residual_delta_heading_relax)
	                      << ",\"attack_residual_safety_gate_ratio\":" << safe_json(result.attack_residual_safety_gate_ratio)
	                      << ",\"mean_attack_effective_heading_error\":" << safe_json(result.mean_attack_effective_heading_error)
	                      << ",\"last_attack_residual_delta_v_ref\":" << safe_json(result.last_attack_residual_delta_v_ref)
	                      << ",\"last_attack_residual_delta_steer_ff\":" << safe_json(result.last_attack_residual_delta_steer_ff)
	                      << ",\"last_attack_residual_delta_ax_bias\":" << safe_json(result.last_attack_residual_delta_ax_bias)
	                      << ",\"last_attack_residual_delta_heading_relax\":" << safe_json(result.last_attack_residual_delta_heading_relax)
	                      << ",\"last_attack_residual_safety_gate_active\":" << (result.last_attack_residual_safety_gate_active ? "true" : "false")
	                      << ",\"last_attack_raw_heading_error\":" << safe_json(result.last_attack_raw_heading_error)
	                      << ",\"last_attack_effective_heading_error\":" << safe_json(result.last_attack_effective_heading_error)
	                      << ",\"last_attack_ax_before_residual\":" << safe_json(result.last_attack_ax_before_residual)
	                      << ",\"last_attack_ax_after_residual\":" << safe_json(result.last_attack_ax_after_residual)
	                      << ",\"mean_accel_headroom\":" << safe_json(result.mean_accel_headroom)
	                      << ",\"healthy_underspeed_ratio\":" << safe_json(result.healthy_underspeed_ratio)
	                      << ",\"last_longitudinal_mode\":\"" << escape_json_string(result.last_longitudinal_mode) << "\""
	                      << ",\"last_track_phase\":\"" << escape_json_string(result.last_track_phase) << "\""
	                      << ",\"last_attack_segment_id\":" << result.last_attack_segment_id
	                      << ",\"last_attack_allowed\":" << (result.last_attack_allowed ? "true" : "false")
	                      << ",\"last_attack_static_speed_cap\":" << safe_json(result.last_attack_static_speed_cap)
	                      << ",\"last_attack_dynamic_speed_cap\":" << safe_json(result.last_attack_dynamic_speed_cap)
	                      << ",\"mean_phase_speed_bias\":" << safe_json(result.mean_phase_speed_bias)
	                      << ",\"mean_phase_margin_bias\":" << safe_json(result.mean_phase_margin_bias)
	                      << ",\"mean_phase_progress_bias\":" << safe_json(result.mean_phase_progress_bias)
	                      << ",\"last_attack_all_rejected\":" << (result.last_attack_all_rejected ? "true" : "false")
	                  << ",\"min_steering_actuator_speed_cap\":" << safe_json(result.min_steering_actuator_speed_cap)
                  << ",\"min_steering_reach_speed_cap\":" << safe_json(result.min_steering_reach_speed_cap)
                  << ",\"min_track_margin_speed_cap\":" << safe_json(result.min_track_margin_speed_cap)
                  << ",\"min_velocity_profile_steering_cap\":" << safe_json(result.min_velocity_profile_steering_cap)
                  << ",\"velocity_profile_steering_cap_ratio\":" << safe_json(result.velocity_profile_steering_cap_ratio)
                  << ",\"velocity_profile_uses_planned_line\":" << (result.velocity_profile_uses_planned_line ? "true" : "false")
                  << ",\"velocity_profile_source\":\"" << escape_json_string(result.velocity_profile_source) << "\""
                  << ",\"velocity_profile_path\":\"" << escape_json_string(result.velocity_profile_path) << "\""
                  << ",\"velocity_profile_fingerprint\":\"" << escape_json_string(result.velocity_profile_fingerprint) << "\""
                  << ",\"velocity_profile_point_count\":" << result.velocity_profile_point_count
                  << ",\"max_abs_phi_twist\":" << safe_json(result.max_abs_phi_twist)
                  << ",\"max_abs_dphi_twist\":" << safe_json(result.max_abs_dphi_twist)
                  << ",\"max_track_limit_excess\":" << safe_json(result.max_track_limit_excess)
                  << ",\"section_straight_ratio\":" << safe_json(result.section_straight_ratio)
                  << ",\"section_entry_ratio\":" << safe_json(result.section_entry_ratio)
                  << ",\"section_apex_ratio\":" << safe_json(result.section_apex_ratio)
                  << ",\"section_exit_ratio\":" << safe_json(result.section_exit_ratio)
                  << ",\"dominant_loss\":\"" << escape_json_string(result.dominant_loss) << "\""
                  << ",\"dominant_loss_none_ratio\":" << safe_json(result.dominant_loss_none_ratio)
                  << ",\"dominant_loss_steering_saturation_ratio\":" << safe_json(result.dominant_loss_steering_saturation_ratio)
                  << ",\"dominant_loss_steering_lag_ratio\":" << safe_json(result.dominant_loss_steering_lag_ratio)
                  << ",\"dominant_loss_track_margin_ratio\":" << safe_json(result.dominant_loss_track_margin_ratio)
                  << ",\"dominant_loss_yaw_mismatch_ratio\":" << safe_json(result.dominant_loss_yaw_mismatch_ratio)
                  << ",\"dominant_loss_front_slip_ratio\":" << safe_json(result.dominant_loss_front_slip_ratio)
                  << ",\"dominant_loss_chassis_twist_ratio\":" << safe_json(result.dominant_loss_chassis_twist_ratio)
                  << ",\"mpc_solve_count\":" << result.mpc_solve_count
                  << ",\"mpc_avg_solve_time_ms\":" << safe_json(result.mpc_avg_solve_time_ms)
                  << ",\"mpc_max_solve_time_ms\":" << safe_json(result.mpc_max_solve_time_ms)
                  << ",\"mpc_fail_count\":" << result.mpc_fail_count
                  << ",\"actuator_prediction_count\":" << result.actuator_prediction_count
                  << ",\"actuator_prediction_bias\":" << safe_json(result.actuator_prediction_bias)
                  << ",\"actuator_prediction_mae\":" << safe_json(result.actuator_prediction_mae)
                  << ",\"actuator_prediction_rmse\":" << safe_json(result.actuator_prediction_rmse)
                  << ",\"actuator_prediction_max_abs_error\":"
                      << safe_json(result.actuator_prediction_max_abs_error)
                  << "}" << std::endl;
        std::cout << "%%JSON_END%%" << std::endl;
    } else {
        std::cout << "\nSimulation Results:" << std::endl;
        std::cout << "Status: " << result.status << std::endl;
        if (!result.failure_code.empty()) std::cout << "Failure code: " << result.failure_code << std::endl;
        if (!result.failure_message.empty()) std::cout << "Failure message: " << result.failure_message << std::endl;
        std::cout << "Tire model: " << result.tire_model_type;
        if (!result.tire_model_name.empty()) std::cout << " (" << result.tire_model_name << ")";
        std::cout << " runtime=" << result.tire_runtime << std::endl;
        std::cout << "Lap completed: " << (result.lap_completed ? "YES" : "NO") << std::endl;
        if (result.lap_completed) std::cout << "Lap time: " << result.lap_time << " s" << std::endl;
        std::cout << "Max lateral error: " << result.max_lateral_error << " m" << std::endl;
        std::cout << "Steering saturation ratio: " << result.steering_saturation_ratio << std::endl;
        std::cout << "Mean SW error: " << result.mean_abs_sw_error << " rad" << std::endl;
        std::cout << "Driver intervention cost: " << result.driver_intervention_cost
                  << " (speed_loss=" << result.mean_torque_speed_loss
                  << ", steer_cap=" << result.mean_steering_cap_speed_loss
                  << ", margin_cap=" << result.mean_track_margin_speed_loss
                  << ", throttle=" << result.mean_corner_throttle_loss
                  << ", control=" << result.control_intervention_cost
                  << ", vehicle_health=" << result.vehicle_health_penalty
                  << ", masking=" << result.driver_masking_risk
                  << ")" << std::endl;
        std::cout << "Min velocity profile steering cap: "
                  << result.min_velocity_profile_steering_cap << " m/s" << std::endl;
        std::cout << "Velocity profile steering cap ratio: "
                  << result.velocity_profile_steering_cap_ratio << std::endl;
        std::cout << "Velocity profile uses planned line: "
                  << (result.velocity_profile_uses_planned_line ? "YES" : "NO") << std::endl;
        std::cout << "Max frame twist: " << result.max_abs_phi_twist << " rad" << std::endl;
        std::cout << "Max track limit excess: " << result.max_track_limit_excess << " m" << std::endl;
        std::cout << "Driver sections: straight=" << result.section_straight_ratio
                  << ", entry=" << result.section_entry_ratio
                  << ", apex=" << result.section_apex_ratio
                  << ", exit=" << result.section_exit_ratio << std::endl;
        std::cout << "Dominant driver loss: " << result.dominant_loss
                  << " (lag=" << result.dominant_loss_steering_lag_ratio
                  << ", margin=" << result.dominant_loss_track_margin_ratio
                  << ", yaw=" << result.dominant_loss_yaw_mismatch_ratio
                  << ", slip=" << result.dominant_loss_front_slip_ratio
                  << ", sat=" << result.dominant_loss_steering_saturation_ratio
                  << ")" << std::endl;
    }

    return exit_code_for_result(result);
}
