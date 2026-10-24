#pragma once
/// @file pid_driver.hpp
/// @brief PID経路追従ドライバー（Level 1）
///
/// 総合ガイド §15.2 に基づく実装
/// 目標経路からの横偏差とヘディング誤差を入力とするPID制御

#include "../vehicle/vehicle_14dof_new.hpp"
#include "gear_selection.hpp"
#include "../track/track_model.hpp"
#include "../track/frenet.hpp"
#include "../laptime/velocity_profile.hpp"
#include "line_planner.hpp"
#include "steering_actuator_controller.hpp"
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

namespace fsae::driver {

inline bool driver_max_throttle_unlimited() {
    const char* raw = std::getenv("FOPT_UNLIMIT_DRIVER_MAX_THROTTLE");
    if (raw == nullptr) {
        return true;
    }
    return !(std::strcmp(raw, "0") == 0 || std::strcmp(raw, "false") == 0);
}

inline bool driver_corner_throttle_limit_disabled() {
    const char* raw = std::getenv("FOPT_DISABLE_DRIVER_CORNER_THROTTLE_LIMIT");
    if (raw == nullptr) {
        return false;
    }
    return !(std::strcmp(raw, "0") == 0 || std::strcmp(raw, "false") == 0);
}

/// PIDゲイン
struct PIDGains {
    // ステアリング制御（外側ループ）
    double Kp_steer = 0.8;      // 比例ゲイン [rad/m]
    double Kd_steer = 0.3;      // 微分ゲイン [rad·s/m]
    double Ki_steer = 0.05;     // 積分ゲイン [rad/(m·s)]
    double Kh_steer = 0.5;      // ヘディング誤差ゲイン [rad/rad]
    double Kff_steer = 1.0;     // フィードフォワードゲイン [-]

    // ステアリング制御（内側ループ・トルクモード用）
    // ω_n・ζ 正規化モード (inner_loop_auto_tune = true) では
    // sync_steering_contract() が I_steer・C_steer_torque から Kp/Kd を自動計算する。
    // 閉ループ解析: ω_n = sqrt(Kp/I), ζ = (Kd+C)/(2*sqrt(Kp*I))
    // 目標: ω_n_target = 30 rad/s (rise ≈ 92 ms), ζ_target = 0.70
    // → Kp = ω_n² * I,  Kd = 2*ζ*ω_n*I - C  (C = C_steer_torque)
    double Kp_steer_inner = 50.0;   // 比例ゲイン [Nm/rad]（auto_tune 時は上書き）
    double Kd_steer_inner = 1.5;    // 微分ゲイン [Nm·s/rad]（auto_tune 時は上書き）
    double Ki_steer_inner = 5.0;    // 積分ゲイン [Nm/(rad*s)]
    bool torque_mode = false;       // トルクモードフラグ
    bool inner_loop_auto_tune = true; // ω_n/ζ ベース自動ゲイン正規化
    double inner_loop_omega_n = 30.0; // 目標固有振動数 [rad/s] (rise ≈ π/(2*ω_n))
    double inner_loop_zeta   = 0.70;  // 目標減衰係数 [-]
    double inner_loop_ki_override = -1.0; // >=0 の場合、auto_tune時もKiだけ手動上書き [Nm/(rad*s)]
    double R_steer = 4.5;           // ステアリングギア比 [-]
    double max_steer_torque = 25.0; // 最大ドライバートルク [Nm]
    double torque_target_speed_scale = 0.55; // トルクモード時の目標速度倍率 [-]
    double torque_outer_loop_scale = 0.8;    // トルクモード時のheading/FF倍率 [-]
    double max_tire_steer_rate = 3.0;        // 目標タイヤ舵角レート上限 [rad/s]
    double torque_steer_rate_reference_torque = 25.0; // レート自動スケール基準トルク [Nm]
    double torque_steer_rate_reference_inertia = 0.02; // レート自動スケール基準慣性 [kg*m^2]
    double torque_steer_rate_authority_exponent = 0.5; // 権限比→レート比の指数 [-]
    double torque_steer_rate_min_scale = 0.65;  // レート自動スケール下限 [-]
    double torque_steer_rate_max_scale = 1.35;  // レート自動スケール上限 [-]
    double torque_lag_speed_penalty = 0.35;  // ステア追従遅れによる追加減速率 [-]
    double torque_saturation_speed_penalty = 0.15; // トルク飽和による追加減速率 [-]
    double chassis_twist_speed_penalty = 0.10;     // フレーム捻り状態による追加減速率 [-]
    double torque_speed_cap_preview_distance = 8.0; // legacy: preview_time<=0時の固定先読み距離 [m]
    double understeer_gradient = 0.002;             // アンダーステア勾配 [rad/(m/s^2)]
    double torque_speed_cap_preview_time = 0.30;    // 操舵到達性の時間ベース先読み係数 [s]
    double torque_speed_cap_min_preview_distance = 3.0; // 操舵到達性の最小先読み距離 [m]
    double torque_speed_cap_max_preview_distance = 25.0; // 操舵到達性の最大先読み距離 [m]
    double torque_lag_lead_gain = 0.0;             // 実舵遅れを見越して目標SW角を進める係数 [-]
    double torque_lag_lead_time = 0.0;             // 0以下なら inner_loop_omega_n から自動推定 [s]
    double torque_lag_lead_max_tire_angle = 0.06;  // lead補正の最大タイヤ舵角相当 [rad]
    double torque_speed_recovery_gain = 0.0;       // 健全な区間で速度倍率を1.0へ戻す強さ [-]
    double torque_speed_recovery_curvature = 0.012; // recoveryを許す曲率スケール [1/m]
    double torque_speed_recovery_lag_ratio = 0.12;  // recoveryを許す操舵遅れ比 [-]
    double torque_speed_recovery_lateral_error = 0.50; // recoveryを許す横偏差スケール [m]
    double torque_speed_recovery_heading_error = 0.12; // recoveryを許すheading誤差スケール [rad]
    double torque_speed_recovery_preview_gap = 0.08;   // 先読み必要舵角gapの許容スケール [rad]
    double torque_speed_recovery_reach_time = 0.04;    // 先読み必要舵角への到達時間スケール [s]
    double torque_reach_speed_cap = 10.0;              // 操舵到達遅れ時の目標速度下限 cap [m/s]
    double torque_reach_time_start = 0.13;             // 到達時間capの介入開始 [s]
    double torque_reach_time_full = 0.22;              // 到達時間capの最大介入 [s]
    double torque_reach_outward_rate_start = 1.2;      // 外向き移動速度capの介入開始 [m/s]
    double torque_reach_outward_rate_full = 3.0;       // 外向き移動速度capの最大介入 [m/s]
    double torque_reach_min_preview_gap = 0.18;        // 到達時間capを許可する最小先読み舵角gap [rad]
    double track_margin_speed_cap = 8.0;            // 境界マージン不足時の速度上限 [m/s]
    double track_margin_speed_release = 1.0;        // 上限を解放する境界マージン [m]
    double track_margin_preview_time = 0.0;         // 外向き走行時の予測マージン時間 [s]

    double torque_stability_vy_threshold = 1.0;     // 横滑り速度しきい値 [m/s]
    double torque_stability_vy_gain = 0.20;         // 横滑り速度ペナルティゲイン [s/m]
    double torque_stability_yaw_threshold = 0.20;   // ヨーレート偏差しきい値 [rad/s]
    double torque_stability_yaw_gain = 0.50;        // ヨーレート偏差ペナルティゲイン [s/rad]
    double torque_stability_slip_threshold = 0.08;  // フロントスリップ角しきい値 [rad] (約4.5deg)
    double torque_stability_slip_gain = 2.0;        // フロントスリップ角ペナルティゲイン [1/rad]
    double distance_cg_front = 0.73;                // CGから前軸までの距離 [m]

    // 速度制御
    double Kp_speed = 0.8;      // 比例ゲイン [1/s]（スロットル側に適用）
    double Ki_speed = 0.15;     // 積分ゲイン [1/s²]
    // ブレーキ感度: Kp_speed × brake_gain_ratio がブレーキ側の実効ゲイン
    // = 0.10 にすると Kp_speed=0.8 で実効ブレーキゲイン0.08（旧と同等）
    // これにより高Kp_speedのまま急ブレーキを防げる
    double brake_gain_ratio = 0.10; // ブレーキ感度係数 [-] (旧Kp_speed/Kp_speed = 0.08/0.8)

    // リミット
    double max_steer = 0.35;    // 最大舵角 [rad] (~20deg)
    double max_throttle = 1.0;  // 最大スロットル [-]
    double max_brake = 1.0;     // 最大ブレーキ [-]

    // フィードフォワード用
    double wheelbase = 1.53;    // ホイールベース [m]

    // アンチワインドアップ
    double integral_limit_steer = 0.5;  // ステア積分リミット [rad]
    double integral_limit_speed = 2.0;  // 速度積分リミット [m/s]

    // コーナリング時スロットル制限: 合成スリップによる横グリップ喪失を防ぐ
    // 高Kp_speedでスロットルが過剰になるコーナーを補正
    // 旧0.1（10%減）→ 0.45（最大舵角時45%減）に増加
    double corner_throttle_reduction = 0.45;
    double corner_throttle_torque_reference = 0.0;  // [Nm], <=0 uses max_steer_torque
    double corner_throttle_lateral_utilization_start = 0.75; // |ay|/(mu_y*g) で介入開始 [-]
    double corner_throttle_lateral_utilization_full = 1.15;  // |ay|/(mu_y*g) で最大介入 [-]
    double corner_throttle_drive_slip_start = 0.08;          // 駆動輪正スリップで介入開始 [-]
    double corner_throttle_drive_slip_full = 0.20;           // 駆動輪正スリップで最大介入 [-]

    // 目標速度の先読み: 未来窓の最小速度を使って、S字や急旋回入口での
    // 過大な進入速度を抑える。固定点 lookahead と異なり、
    // 「未来の速い区間」を先食いして加速しすぎることは避けられる。
    double target_speed_preview_distance = 0.0;  // [m]

    // 操舵用の先読み: 現在点ではなく少し先の曲率/headingを使い、
    // 高速域でのコーナー進入遅れを抑える。
    double steering_preview_distance = 0.0;       // [m], steering_preview_time<=0時の固定先読み距離
    double steering_preview_time = 0.0;           // [s], >0なら車速×時間で外側ループ先読みを決める
    double steering_preview_min_distance = 0.0;   // [m], 時間ベース先読みの下限
    double steering_preview_max_distance = 30.0;  // [m], 時間ベース先読みの上限
    double Kpp_steer = 0.0;                       // pure-pursuit補助ゲイン [-]
    double steering_preview_authority_threshold = 0.0;  // torque mode: preview requires authority scale >= threshold

    // トラック幅内の幾何ラインplanner。既定はセンターライン追従。
    BoundaryLinePlannerConfig line_planner;
};

/// PID状態（積分項保持用）
struct PIDState {
    double integral_lateral_error = 0.0;  // 横偏差の積分 [m·s]
    double integral_speed_error = 0.0;    // 速度誤差の積分 [m/s·s]
    double prev_lateral_error = 0.0;      // 前回の横偏差 [m]
    double prev_time = 0.0;               // 前回の時刻 [s]
};

/// PID診断情報（最後のcompute_controlで更新）
struct PIDDebugInfo {
    double s = 0.0;
    double n = 0.0;
    double d_heading = 0.0;
    double kappa = 0.0;
    double target_lateral_offset = 0.0;
    double line_curvature_now = 0.0;
    double line_curvature_ahead = 0.0;
    double line_curvature_behind = 0.0;
    double target_heading_error = 0.0;
    double target_curvature = 0.0;
    double line_heading_offset = 0.0;
    double current_speed = 0.0;
    double target_speed_base = 0.0;
    double target_speed_preview = 0.0;
    double target_speed_final = 0.0;
    double steering_preview_distance_used = 0.0;
    double delta_target = 0.0;
    double target_sw = 0.0;
    double delta_sw = 0.0;
    double ddelta_sw = 0.0;
    double required_delta_preview = 0.0;
    double actual_delta_tire = 0.0;
    double predicted_delta_tire = 0.0;
    double preview_delta_gap = 0.0;
    double sw_error = 0.0;
    double torque_lag_lead_sw = 0.0;
    double T_SAT_feedforward = 0.0;
    double T_driver_raw = 0.0;
    double T_driver_clamped = 0.0;
    double torque_headroom = 1.0;
    double time_to_reach_delta = 0.0;
    double steering_saturation = 0.0;
    double torque_speed_multiplier = 1.0;
    double torque_speed_recovery_ratio = 0.0;
    double steering_lag_ratio = 0.0;
    double chassis_twist_risk = 0.0;
    double effective_max_tire_steer_rate = 0.0;
    double steering_actuator_speed_cap = 0.0;
    double steering_reach_speed_cap = std::numeric_limits<double>::infinity();
    double track_margin_speed_cap = 0.0;
    double track_margin_effective_margin = std::numeric_limits<double>::infinity();
    double track_margin_outward_rate = 0.0;
    double corner_throttle_steer_ratio = 0.0;
    double corner_throttle_lateral_utilization = 0.0;
    double corner_throttle_lateral_ratio = 0.0;
    double corner_throttle_drive_slip = 0.0;
    double corner_throttle_drive_slip_ratio = 0.0;
    double corner_throttle_limit_ratio = 0.0;
    double corner_throttle_scale = 1.0;
    double phi_twist = 0.0;
    double dphi_twist = 0.0;
    double T_SAT_total = 0.0;
    double stability_vy_penalty = 0.0;
    double stability_yaw_penalty = 0.0;
    double stability_slip_penalty = 0.0;
    double stability_penalty = 0.0;
    double estimated_front_slip = 0.0;
};

/// PID経路追従ドライバー
class PIDPathFollowingDriver {
public:
    /// @param track トラックモデル
    /// @param velocity_profile 速度プロファイル
    /// @param gains PIDゲイン
    PIDPathFollowingDriver(
        const track::CurvatureBasedTrack& track,
        const std::vector<laptime::VelocityPoint>& velocity_profile,
        const PIDGains& gains = PIDGains(),
        const vehicle::VehicleParams& vehicle_params = vehicle::VehicleParams::default_fsae())
        : track_(track)
        , velocity_profile_(velocity_profile)
        , gains_(gains)
        , vehicle_params_(vehicle_params)
        , converter_(track)
    {
        sync_steering_contract();
        rebuild_planned_line_geometry_cache();
    }

    /// 制御入力を計算
    /// @param vehicle_state 車両状態
    /// @param current_time 現在時刻 [s]
    /// @return ドライバー入力
    vehicle::DriverInput compute_control(
        const vehicle::State14DOF& vehicle_state,
        double current_time,
        double T_SAT_total = 0.0)
    {
        vehicle::DriverInput input;

        // 時間差分を先に計算（ステアリングと速度制御で共有）
        double dt = (state_.prev_time > 0.0) ? (current_time - state_.prev_time) : 0.001;
        dt = std::max(dt, 0.0001);  // ゼロ除算防止

        // 1. 車両位置をFrenet座標に変換（NEDからMap座標へ変換）
        // Vehicle X (North) = Map Y
        // Vehicle Y (East) = Map X
        // Vehicle Psi (CW) = pi/2 - Map Theta (CCW)
        double map_x = vehicle_state.y;
        double map_y = vehicle_state.x;
        double map_theta = M_PI / 2.0 - vehicle_state.psi;

        auto frenet = converter_.global_to_frenet(
            map_x,
            map_y,
            map_theta
        );

        // 2. 目標速度を取得
        //    以前の固定 6 m lookahead は autocross で過大な目標速度を生み、
        //    現在位置より先のコーナー速度を常に先食いしてしまっていた。
        const double target_speed_base = get_target_speed(frenet.s);
        double target_speed = target_speed_base;
        if (gains_.target_speed_preview_distance > 1e-6) {
            target_speed = std::min(
                target_speed,
                get_preview_min_target_speed(frenet.s, gains_.target_speed_preview_distance)
            );
        }
        const double target_speed_preview = target_speed;

        // 3. ステアリング制御（PID + フィードフォワード）
        const auto line_geometry = compute_tracking_line_geometry(frenet.s);
        double delta_target = compute_steering_control(
            frenet,
            vehicle_state,
            dt,
            line_geometry);

        last_debug_ = PIDDebugInfo{};
        last_debug_.s = frenet.s;
        last_debug_.n = frenet.n;
        last_debug_.d_heading = frenet.d_heading;
        last_debug_.kappa = line_geometry.curvature;
        last_debug_.target_lateral_offset = line_geometry.target_n;
        last_debug_.line_curvature_now = line_geometry.plan.curvature_now;
        last_debug_.line_curvature_ahead = line_geometry.plan.curvature_ahead;
        last_debug_.line_curvature_behind = line_geometry.plan.curvature_behind;
        last_debug_.target_heading_error = normalize_angle(map_theta - line_geometry.heading);
        last_debug_.target_curvature = line_geometry.curvature;
        last_debug_.line_heading_offset = line_geometry.heading_offset;
        last_debug_.current_speed = vehicle_state.speed();
        last_debug_.target_speed_base = target_speed_base;
        last_debug_.target_speed_preview = target_speed_preview;
        last_debug_.steering_preview_distance_used =
            compute_effective_steering_preview_distance(vehicle_state.speed());
        last_debug_.delta_target = delta_target;
        last_debug_.phi_twist = vehicle_state.phi_twist;
        last_debug_.dphi_twist = vehicle_state.dphi_twist;
        last_debug_.T_SAT_total = T_SAT_total;

        if (gains_.torque_mode) {
            const double effective_tire_steer_rate = compute_effective_max_tire_steer_rate();
            const auto actuator_output = compute_steering_actuator_control(
                make_steering_actuator_config(effective_tire_steer_rate),
                steering_actuator_state_,
                SteeringActuatorCommand{
                    delta_target,
                    vehicle_state.delta_sw,
                    vehicle_state.ddelta_sw,
                    dt,
                    T_SAT_total,
                });
            input.steering = actuator_output.steering_torque;

            last_debug_.target_sw = actuator_output.target_sw;
            last_debug_.delta_sw = vehicle_state.delta_sw;
            last_debug_.ddelta_sw = vehicle_state.ddelta_sw;
            last_debug_.torque_lag_lead_sw = actuator_output.lag_lead_sw;
            last_debug_.effective_max_tire_steer_rate = effective_tire_steer_rate;
            last_debug_.actual_delta_tire = actuator_output.actual_delta_tire;
            last_debug_.predicted_delta_tire =
                compute_predicted_tire_angle(
                    delta_target,
                    vehicle_state.delta_sw,
                    vehicle_state.ddelta_sw);
            last_debug_.required_delta_preview =
                compute_required_preview_tire_angle(
                    frenet.s, last_debug_.actual_delta_tire, last_debug_.current_speed);
            last_debug_.preview_delta_gap =
                last_debug_.required_delta_preview - last_debug_.actual_delta_tire;
            last_debug_.time_to_reach_delta =
                compute_time_to_reach_delta(
                    last_debug_.required_delta_preview,
                    vehicle_state.delta_sw,
                    vehicle_state.ddelta_sw);
            last_debug_.sw_error = actuator_output.sw_error;
            last_debug_.T_SAT_feedforward = actuator_output.T_SAT_feedforward;
            last_debug_.T_driver_raw = actuator_output.T_driver_raw;
            last_debug_.T_driver_clamped = actuator_output.T_driver_clamped;
            last_debug_.torque_headroom = actuator_output.torque_headroom;
            last_debug_.steering_saturation = actuator_output.steering_saturation;
            last_debug_.steering_lag_ratio = actuator_output.steering_lag_ratio;
        } else {
            // 角度モード
            input.steering = delta_target;
            last_debug_.T_driver_raw = input.steering;
            last_debug_.T_driver_clamped = input.steering;
            last_debug_.actual_delta_tire = delta_target;
        }

        if (gains_.torque_mode) {
            const double speed_multiplier = compute_torque_mode_speed_multiplier(last_debug_, vehicle_state);
            target_speed *= speed_multiplier;
            last_debug_.torque_speed_multiplier = speed_multiplier;
        }
        last_debug_.target_speed_final = target_speed;

        // 4. 速度制御（PI + ブレーキ）
        double current_speed = vehicle_state.speed();
        auto [throttle, brake] = compute_speed_control(current_speed, target_speed, dt);
        throttle = std::min(throttle, compute_progressive_throttle_limit(frenet, current_speed, target_speed));

        // 5. コーナリング時スロットル制限は維持し、preset 由来の max_throttle だけ撤廃する。
        if (driver_corner_throttle_limit_disabled()) {
            input.throttle = throttle;
            last_debug_.corner_throttle_steer_ratio = 0.0;
            last_debug_.corner_throttle_lateral_utilization = 0.0;
            last_debug_.corner_throttle_lateral_ratio = 0.0;
            last_debug_.corner_throttle_drive_slip = 0.0;
            last_debug_.corner_throttle_drive_slip_ratio = 0.0;
            last_debug_.corner_throttle_limit_ratio = 0.0;
            last_debug_.corner_throttle_scale = 1.0;
        } else {
            const double steer_metric = std::abs(last_debug_.actual_delta_tire);
            const double steer_max_for_ratio = std::max(gains_.max_steer, 1e-6);
            const double steer_ratio = std::clamp(steer_metric / steer_max_for_ratio, 0.0, 1.0);
            const double lateral_utilization =
                compute_cornering_lateral_utilization(last_debug_, vehicle_state);
            const double lateral_ratio = ramp_ratio(
                lateral_utilization,
                gains_.corner_throttle_lateral_utilization_start,
                gains_.corner_throttle_lateral_utilization_full);
            const double drive_slip = compute_rear_drive_slip_ratio(vehicle_state);
            const double drive_slip_ratio = ramp_ratio(
                drive_slip,
                gains_.corner_throttle_drive_slip_start,
                gains_.corner_throttle_drive_slip_full);
            const double limit_ratio = std::max({steer_ratio, lateral_ratio, drive_slip_ratio});
            double throttle_scale = std::max(0.0, 1.0 - limit_ratio * gains_.corner_throttle_reduction);
            input.throttle = throttle * throttle_scale;
            last_debug_.corner_throttle_steer_ratio = steer_ratio;
            last_debug_.corner_throttle_lateral_utilization = lateral_utilization;
            last_debug_.corner_throttle_lateral_ratio = lateral_ratio;
            last_debug_.corner_throttle_drive_slip = drive_slip;
            last_debug_.corner_throttle_drive_slip_ratio = drive_slip_ratio;
            last_debug_.corner_throttle_limit_ratio = limit_ratio;
            last_debug_.corner_throttle_scale = throttle_scale;
        }
        input.brake = brake;

        // 6. DRS制御: 直線・高速・低舵角時だけ要求し、最終安全判定はDRSSystemに任せる。
        const double local_curvature = track_.get_curvature(frenet.s);
        const double drs_steer_metric = gains_.torque_mode
            ? std::max(std::abs(last_debug_.actual_delta_tire), std::abs(delta_target))
            : std::abs(input.steering);
        const double drs_lateral_error = std::abs(frenet.n - line_geometry.target_n);
        input.drs_button = current_speed > 20.0
                         && std::abs(local_curvature) < 0.015
                         && drs_steer_metric < 0.05
                         && drs_lateral_error < 0.35
                         && std::abs(frenet.d_heading) < 0.08;

        // 7. ギア選択（車両のギア比・タイヤ半径・レッドラインに連動）
        input.gear = select_gear(current_speed);

        // 時刻を更新（一箇所で管理）
        state_.prev_time = current_time;

        return input;
    }

    /// PID状態をリセット
    void reset() {
        state_ = PIDState();
        steering_actuator_state_.reset();
        last_debug_ = PIDDebugInfo{};
    }

    /// PIDゲインを設定
    void set_gains(const PIDGains& gains) {
        gains_ = gains;
        sync_steering_contract();
        rebuild_planned_line_geometry_cache();
    }

    const PIDDebugInfo& get_last_debug() const {
        return last_debug_;
    }

    /// 現在のFrenet座標を取得（デバッグ用）
    track::FrenetCoordinate get_current_frenet(const vehicle::State14DOF& vehicle_state) const {
        return converter_.global_to_frenet(vehicle_state.y, vehicle_state.x, M_PI / 2.0 - vehicle_state.psi);
    }

    /// 目標速度を取得
    double get_target_speed(double s) const {
        if (velocity_profile_.empty()) {
            return 15.0;  // デフォルト速度
        }

        // 線形補間
        if (s <= velocity_profile_.front().s) {
            return velocity_profile_.front().v;
        }
        if (s >= velocity_profile_.back().s) {
            return velocity_profile_.back().v;
        }

        for (size_t i = 0; i < velocity_profile_.size() - 1; ++i) {
            const auto& p0 = velocity_profile_[i];
            const auto& p1 = velocity_profile_[i + 1];

            if (s >= p0.s && s < p1.s) {
                double alpha = (s - p0.s) / (p1.s - p0.s);
                return p0.v + alpha * (p1.v - p0.v);
            }
        }

        return velocity_profile_.back().v;
    }

private:
    double wrap_track_s(double s) const {
        double track_length = track_.length();
        if (!track_.is_closed_loop() || track_length <= 1e-6) {
            return s;
        }

        while (s < 0.0) {
            s += track_length;
        }
        while (s > track_length) {
            s -= track_length;
        }
        return s;
    }

    static double ramp_ratio(double value, double start, double full) {
        if (full <= start + 1e-9) {
            return value >= full ? 1.0 : 0.0;
        }
        return std::clamp((value - start) / (full - start), 0.0, 1.0);
    }

    double compute_cornering_lateral_utilization(
        const PIDDebugInfo& debug,
        const vehicle::State14DOF& vehicle_state) const
    {
        const double mu_y = std::max(vehicle_params_.tire.mu_y_max, 0.1);
        const double speed = std::max(debug.current_speed, 0.0);
        const double ay_demand = speed * speed * std::abs(debug.target_curvature);
        const double ay_yaw = std::abs(vehicle_state.vx * vehicle_state.r);
        constexpr double kGravity = 9.80665;
        return std::max(ay_demand, ay_yaw) / (mu_y * kGravity);
    }

    double compute_rear_drive_slip_ratio(const vehicle::State14DOF& vehicle_state) const {
        const double radius = std::max(vehicle_params_.tire.radius, 1e-6);
        const double v_ref = std::max(std::abs(vehicle_state.vx), vehicle_params_.tire.v_N);
        double worst_drive_slip = 0.0;
        for (int wheel : {vehicle::RL, vehicle::RR}) {
            const double wheel_speed = vehicle_state.omega[static_cast<std::size_t>(wheel)] * radius;
            const double denom = std::max({std::abs(wheel_speed), v_ref, 1e-6});
            const double slip = (wheel_speed - vehicle_state.vx) / denom;
            worst_drive_slip = std::max(worst_drive_slip, slip);
        }
        return std::clamp(worst_drive_slip, 0.0, 1.0);
    }

    double get_preview_min_target_speed(double s, double preview_distance) const {
        double min_speed = get_target_speed(s);
        if (preview_distance <= 1e-6 || velocity_profile_.empty()) {
            return min_speed;
        }

        constexpr double kPreviewStep = 0.5;  // [m]
        for (double ds = kPreviewStep; ds <= preview_distance; ds += kPreviewStep) {
            double sample_s = wrap_track_s(s + ds);
            min_speed = std::min(min_speed, get_target_speed(sample_s));
        }
        return min_speed;
    }

    SteeringActuatorConfig make_steering_actuator_config(double effective_tire_steer_rate) const {
        SteeringActuatorConfig config;
        config.Kp_inner = gains_.Kp_steer_inner;
        config.Ki_inner = gains_.Ki_steer_inner;
        config.Kd_inner = gains_.Kd_steer_inner;
        config.R_steer = gains_.R_steer;
        config.max_tire_angle = gains_.max_steer;
        config.max_driver_torque = gains_.max_steer_torque;
        config.max_tire_steer_rate = effective_tire_steer_rate;
        config.inner_loop_omega_n = gains_.inner_loop_omega_n;
        config.inner_loop_zeta = gains_.inner_loop_zeta;
        config.lag_lead_gain = gains_.torque_lag_lead_gain;
        config.lag_lead_time = gains_.torque_lag_lead_time;
        config.lag_lead_max_tire_angle = gains_.torque_lag_lead_max_tire_angle;
        config.integral_limit = 0.5;
        return config;
    }

    double compute_steering_lag_ratio(double sw_error) const {
        const double sw_range = std::max(gains_.max_steer * gains_.R_steer, 1e-6);
        return std::clamp(std::abs(sw_error) / sw_range, 0.0, 1.0);
    }

    double compute_predicted_tire_angle(
        double target_tire_angle,
        double delta_sw,
        double ddelta_sw) const
    {
        const double R = std::max(gains_.R_steer, 1e-6);
        const double effective_tire_rate = compute_effective_max_tire_steer_rate();
        const auto predicted = predict_steering_actuator_response(
            make_steering_actuator_config(effective_tire_rate),
            SteeringActuatorPredictionState{delta_sw / R, ddelta_sw / R},
            target_tire_angle,
            compute_torque_lag_lead_time());
        return predicted.tire_angle;
    }

    double compute_torque_lag_lead_time() const {
        if (gains_.torque_lag_lead_time > 1e-6) {
            return std::clamp(gains_.torque_lag_lead_time, 0.0, 0.150);
        }
        return std::clamp(
            1.0 / std::max(gains_.inner_loop_omega_n, 1e-6),
            0.010,
            0.080
        );
    }

    std::pair<double, double> compute_lag_compensated_target_sw(
        double desired_sw,
        double delta_sw,
        double ddelta_sw) const
    {
        const double gain = std::max(gains_.torque_lag_lead_gain, 0.0);
        const double R = std::max(gains_.R_steer, 1e-6);
        if (!gains_.torque_mode || gain <= 1e-9 || R <= 1e-6) {
            return {desired_sw, 0.0};
        }

        const double prediction_time = compute_torque_lag_lead_time();
        const double predicted_sw = delta_sw + ddelta_sw * prediction_time;
        const double max_lead_sw = std::max(gains_.torque_lag_lead_max_tire_angle, 0.0) * R;
        const double lead_sw = std::clamp(
            gain * (desired_sw - predicted_sw),
            -max_lead_sw,
            max_lead_sw
        );
        const double max_sw = std::max(gains_.max_steer, 0.0) * R;
        const double compensated_sw = std::clamp(desired_sw + lead_sw, -max_sw, max_sw);
        return {compensated_sw, compensated_sw - desired_sw};
    }

    double compute_torque_speed_cap_preview_distance(double current_speed) const {
        if (gains_.torque_speed_cap_preview_time <= 1e-9) {
            return std::max(gains_.torque_speed_cap_preview_distance, 0.0);
        }

        const double min_distance = std::max(gains_.torque_speed_cap_min_preview_distance, 0.0);
        const double max_distance = std::max(gains_.torque_speed_cap_max_preview_distance, min_distance);
        return std::clamp(
            gains_.torque_speed_cap_preview_time * std::max(current_speed, 0.0),
            min_distance,
            max_distance);
    }

    double compute_required_tire_angle_for_geometry(
        const PlannedLineGeometry& geometry,
        double target_speed) const
    {
        const double speed = std::max(target_speed, 0.0);
        const double target_ay = speed * speed * geometry.curvature;
        const double required_tire_angle =
            -(gains_.wheelbase * geometry.curvature + gains_.understeer_gradient * target_ay);
        return std::clamp(required_tire_angle, -gains_.max_steer, gains_.max_steer);
    }

    double compute_required_preview_tire_angle(
        double s,
        double current_tire_angle,
        double current_speed) const
    {
        const double preview_distance = compute_torque_speed_cap_preview_distance(current_speed);
        const auto current_geometry = compute_tracking_line_geometry(s);
        const double target_speed = std::max(current_speed, 1.0);
        double best_required = compute_required_tire_angle_for_geometry(current_geometry, target_speed);
        if (preview_distance <= 1e-6) {
            return best_required;
        }

        double best_gap = std::abs(best_required - current_tire_angle);
        constexpr double kStep = 1.0;  // [m]
        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            const auto geometry = compute_tracking_line_geometry(s + ds);
            const double required = compute_required_tire_angle_for_geometry(geometry, target_speed);
            const double gap = std::abs(required - current_tire_angle);
            if (gap > best_gap) {
                best_gap = gap;
                best_required = required;
            }
        }
        return best_required;
    }

    double compute_time_to_reach_delta(
        double target_tire_angle,
        double delta_sw,
        double ddelta_sw) const
    {
        const double R = std::max(gains_.R_steer, 1e-6);
        const double rate = compute_effective_max_tire_steer_rate();
        constexpr double kDeadband = 0.02;  // [rad]
        constexpr double kMaxReachTime = 1.0; // [s]
        const double reach_time = estimate_steering_actuator_reach_time(
            make_steering_actuator_config(rate),
            SteeringActuatorPredictionState{delta_sw / R, ddelta_sw / R},
            target_tire_angle,
            kDeadband,
            kMaxReachTime);
        return std::isfinite(reach_time) ? reach_time : kMaxReachTime;
    }

    double compute_torque_headroom(double applied_torque) const {
        const double limit = std::max(gains_.max_steer_torque, 1e-6);
        return std::clamp((limit - std::abs(applied_torque)) / limit, 0.0, 1.0);
    }

    double compute_chassis_twist_risk(const PIDDebugInfo& debug) const {
        const double twist_ratio = std::abs(debug.phi_twist) / 0.04;       // ~2.3deg
        const double twist_rate_ratio = std::abs(debug.dphi_twist) / 1.0;  // rad/s
        return std::clamp(std::max(twist_ratio, twist_rate_ratio), 0.0, 1.0);
    }

    double compute_steering_authority_speed_scale() const {
        const auto& steering = vehicle_params_.steering;
        const double available_torque = std::max(
            steering.T_driver_max - std::max(steering.T_fric, 0.0), 1e-3);
        const double authority = available_torque / std::max(steering.I_steer, 1e-6);

        // Nominal Phase-A torque-steering plant: T_driver_max=25Nm, T_fric=0.3Nm, I_steer=0.02kgm^2.
        // Lower torque authority should reduce entry speed instead of waiting for off-track recovery.
        constexpr double kNominalAuthority = (25.0 - 0.3) / 0.02;
        const double authority_ratio = std::clamp(authority / kNominalAuthority, 0.0, 1.0);
        return std::clamp(std::pow(authority_ratio, 1.5), 0.65, 1.0);
    }

    double compute_effective_max_tire_steer_rate() const {
        if (!gains_.torque_mode) {
            return gains_.max_tire_steer_rate;
        }

        const auto& steering = vehicle_params_.steering;
        const double reference_torque = std::max(gains_.torque_steer_rate_reference_torque, 1e-6);
        const double reference_inertia = std::max(gains_.torque_steer_rate_reference_inertia, 1e-6);
        const double exponent = std::max(gains_.torque_steer_rate_authority_exponent, 0.0);
        const double friction = std::max(steering.T_fric, 0.0);
        const double reflected_SAT = std::abs(last_debug_.T_SAT_total) / steering.R_steer;
        const double available_torque = std::max(
            steering.T_driver_max - friction - reflected_SAT,
            1e-3);
        const double available_authority = available_torque / std::max(steering.I_steer, 1e-6);
        const double reference_authority = std::max(
            reference_torque - friction, 1e-6) / reference_inertia;
        const double authority_ratio = available_authority / std::max(reference_authority, 1e-6);
        const double raw_scale = std::pow(std::max(authority_ratio, 1e-6), exponent);
        const double min_scale = std::max(gains_.torque_steer_rate_min_scale, 0.0);
        const double max_scale = std::max(gains_.torque_steer_rate_max_scale, min_scale);
        const double rate_scale = std::clamp(raw_scale, min_scale, max_scale);
        return gains_.max_tire_steer_rate * rate_scale;
    }

    double compute_effective_steering_preview_distance(double current_speed) const {
        double preview = std::max(gains_.steering_preview_distance, 0.0);
        if (gains_.steering_preview_time > 1e-9) {
            const double min_distance = std::max(gains_.steering_preview_min_distance, 0.0);
            const double max_distance = std::max(gains_.steering_preview_max_distance, min_distance);
            preview = std::clamp(
                gains_.steering_preview_time * std::max(current_speed, 0.0),
                min_distance,
                max_distance);
        }

        if (!gains_.torque_mode || preview <= 1e-6) {
            return preview;
        }

        const double threshold = std::clamp(gains_.steering_preview_authority_threshold, 0.0, 1.0);
        if (threshold <= 1e-9) {
            return preview;
        }

        const double authority_scale = compute_steering_authority_speed_scale();
        return authority_scale + 1e-9 >= threshold ? preview : 0.0;
    }

    double compute_torque_speed_recovery_ratio(const PIDDebugInfo& debug) const {
        if (gains_.torque_speed_recovery_gain <= 1e-9 || debug.steering_saturation > 0.5) {
            return 0.0;
        }

        const double curvature_scale = std::max(gains_.torque_speed_recovery_curvature, 1e-6);
        const double lag_scale = std::max(gains_.torque_speed_recovery_lag_ratio, 1e-6);
        const double lateral_scale = std::max(gains_.torque_speed_recovery_lateral_error, 1e-6);
        const double heading_scale = std::max(gains_.torque_speed_recovery_heading_error, 1e-6);
        const double preview_gap_scale = std::max(gains_.torque_speed_recovery_preview_gap, 1e-6);
        const double reach_time_scale = std::max(gains_.torque_speed_recovery_reach_time, 1e-6);
        const double lateral_error = debug.n - debug.target_lateral_offset;

        const double curvature_gate = 1.0 - std::clamp(std::abs(debug.kappa) / curvature_scale, 0.0, 1.0);
        const double lag_gate = 1.0 - std::clamp(debug.steering_lag_ratio / lag_scale, 0.0, 1.0);
        const double lateral_gate = 1.0 - std::clamp(std::abs(lateral_error) / lateral_scale, 0.0, 1.0);
        const double heading_gate = 1.0 - std::clamp(std::abs(debug.target_heading_error) / heading_scale, 0.0, 1.0);
        const double preview_gap_gate =
            1.0 - std::clamp(std::abs(debug.preview_delta_gap) / preview_gap_scale, 0.0, 1.0);
        const double reach_time_gate =
            1.0 - std::clamp(debug.time_to_reach_delta / reach_time_scale, 0.0, 1.0);
        const double headroom_gate = std::clamp((debug.torque_headroom - 0.20) / 0.50, 0.0, 1.0);
        const double twist_gate = 1.0 - debug.chassis_twist_risk;

        return std::clamp(
            curvature_gate * lag_gate * lateral_gate * heading_gate
                * preview_gap_gate * reach_time_gate * headroom_gate * twist_gate,
            0.0,
            1.0
        );
    }

    double compute_torque_mode_speed_multiplier(PIDDebugInfo& debug, const vehicle::State14DOF& vehicle_state) const {
        debug.chassis_twist_risk = compute_chassis_twist_risk(debug);
        debug.steering_actuator_speed_cap = compute_steering_actuator_speed_cap(debug);
        const auto track_margin_projection = compute_track_margin_projection(debug);
        debug.track_margin_effective_margin = track_margin_projection.first;
        debug.track_margin_outward_rate = track_margin_projection.second;
        debug.track_margin_speed_cap = compute_track_margin_speed_cap(debug.track_margin_effective_margin,
                                                                      debug.target_speed_preview);
        debug.steering_reach_speed_cap = compute_steering_reach_speed_cap(debug);
        debug.torque_speed_recovery_ratio = compute_torque_speed_recovery_ratio(debug);

        // 横すべり速度 (vy) ペナルティの計算
        const double vy_excess = std::max(0.0, std::abs(vehicle_state.vy) - gains_.torque_stability_vy_threshold);
        debug.stability_vy_penalty = vy_excess * gains_.torque_stability_vy_gain;

        // ヨーレート追従偏差の計算。
        // Track kappa is map-CCW positive, while vehicle psi/r are CW positive:
        // map_theta = pi/2 - psi, so r_ref = -vx*kappa.
        const double r_kinematic = -vehicle_state.vx * debug.target_curvature;
        const double r_err = std::abs(vehicle_state.r - r_kinematic);
        const double r_err_excess = std::max(0.0, r_err - gains_.torque_stability_yaw_threshold);
        debug.stability_yaw_penalty = r_err_excess * gains_.torque_stability_yaw_gain;

        // フロントタイヤスリップ角のキネマティックな推定 (α_f = δ_tire - (vy + a * r) / vx)
        const double delta_tire = debug.actual_delta_tire;
        const double vx_safe = std::max(vehicle_state.vx, 1.0);
        const double alpha_f = delta_tire - (vehicle_state.vy + gains_.distance_cg_front * vehicle_state.r) / vx_safe;
        debug.estimated_front_slip = alpha_f;
        const double slip_excess = std::max(0.0, std::abs(alpha_f) - gains_.torque_stability_slip_threshold);
        debug.stability_slip_penalty = slip_excess * gains_.torque_stability_slip_gain;

        // 安定性ペナルティの統合 (Max-Risk)
        debug.stability_penalty = std::max({debug.stability_vy_penalty, debug.stability_yaw_penalty, debug.stability_slip_penalty});

        double max_penalty = 0.0;
        max_penalty = std::max(max_penalty, gains_.torque_lag_speed_penalty * debug.steering_lag_ratio);
        if (debug.steering_saturation > 0.5) {
            max_penalty = std::max(max_penalty, gains_.torque_saturation_speed_penalty);
        }
        max_penalty = std::max(max_penalty, gains_.chassis_twist_speed_penalty * debug.chassis_twist_risk);
        max_penalty = std::max(max_penalty, debug.stability_penalty);

        double multiplier = gains_.torque_target_speed_scale
            + (1.0 - gains_.torque_target_speed_scale)
            * std::clamp(gains_.torque_speed_recovery_gain, 0.0, 1.0)
            * debug.torque_speed_recovery_ratio;
        multiplier *= compute_steering_authority_speed_scale();
        multiplier *= (1.0 - max_penalty);

        const double speed_cap = std::min({
            debug.steering_actuator_speed_cap,
            debug.steering_reach_speed_cap,
            debug.track_margin_speed_cap,
        });
        if (std::isfinite(speed_cap) && speed_cap > 1e-6 && debug.target_speed_preview > 1e-6) {
            multiplier = std::min(multiplier, speed_cap / debug.target_speed_preview);
        }

        return std::clamp(multiplier, 0.35, 1.0);
    }

    double compute_steering_actuator_speed_cap(const PIDDebugInfo& debug) const {
        const double preview_distance = compute_torque_speed_cap_preview_distance(debug.current_speed);
        const double effective_tire_steer_rate = compute_effective_max_tire_steer_rate();
        if (preview_distance <= 1e-6 || effective_tire_steer_rate <= 1e-6 || gains_.R_steer <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }

        // A-2: 速度 cap スケール係数を内側ループの帯域幅 (ω_n) に比例させる。
        // 基準 ω_n = 30 rad/s では係数 0.85。高帯域 → 上限を緩和、低帯域 → 厳しめ。
        // 係数範囲は [0.55, 1.10] にクランプし、ω_n 依存で ±30% 変動させる。
        const double omega_n_ratio = std::clamp(
            gains_.inner_loop_omega_n / 30.0, 0.65, 1.30);
        const double bandwidth_scale = 0.85 * omega_n_ratio;  // [0.55, 1.10]

        const double current_tire_angle = debug.delta_sw / gains_.R_steer;
        const double current_tire_rate = debug.ddelta_sw / gains_.R_steer;
        const auto prediction_config =
            make_steering_actuator_config(effective_tire_steer_rate);
        double cap = std::numeric_limits<double>::infinity();
        constexpr double kStep = 1.0;       // [m]
        constexpr double kDeadband = 0.02;  // [rad], avoid limiting on tiny curvature noise
        constexpr double kMaxReachTime = 1.0; // [s]

        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            const auto geometry = compute_tracking_line_geometry(debug.s + ds);
            const double target_speed = std::max(debug.current_speed, 1.0);
            const double required_tire_angle =
                compute_required_tire_angle_for_geometry(geometry, target_speed);
            const double angle_gap = std::max(0.0, std::abs(required_tire_angle - current_tire_angle) - kDeadband);
            if (angle_gap <= 1e-9) {
                continue;
            }
            const double reach_time = estimate_steering_actuator_reach_time(
                prediction_config,
                SteeringActuatorPredictionState{current_tire_angle, current_tire_rate},
                required_tire_angle,
                kDeadband,
                kMaxReachTime);
            const double local_cap = std::isfinite(reach_time) && reach_time > 1e-9
                ? bandwidth_scale * ds / reach_time
                : 3.0;
            cap = std::min(cap, local_cap);
        }

        if (!std::isfinite(cap)) {
            return cap;
        }
        return std::clamp(cap, 3.0, 60.0);
    }

    double compute_steering_reach_speed_cap(const PIDDebugInfo& debug) const {
        const double base_cap = std::max(gains_.torque_reach_speed_cap, 0.0);
        if (base_cap <= 1e-6 || debug.target_speed_preview <= base_cap + 1e-6) {
            return std::numeric_limits<double>::infinity();
        }

        if (std::abs(debug.preview_delta_gap) < std::max(gains_.torque_reach_min_preview_gap, 0.0)) {
            return std::numeric_limits<double>::infinity();
        }

        const double reach_ratio = ramp_ratio(
            debug.time_to_reach_delta,
            gains_.torque_reach_time_start,
            gains_.torque_reach_time_full);
        const double outward_ratio = ramp_ratio(
            std::max(debug.track_margin_outward_rate, 0.0),
            gains_.torque_reach_outward_rate_start,
            gains_.torque_reach_outward_rate_full);
        const double margin_ratio =
            1.0 - std::clamp(debug.track_margin_effective_margin
                             / std::max(gains_.track_margin_speed_release, 1e-3),
                             0.0,
                             1.0);
        const double risk = std::clamp(
            std::max(reach_ratio, std::max(outward_ratio, margin_ratio)),
            0.0,
            1.0);
        if (risk <= 1e-9) {
            return std::numeric_limits<double>::infinity();
        }

        const double release = (1.0 - risk) * (1.0 - risk);
        const double cap = base_cap + (debug.target_speed_preview - base_cap) * release;
        return std::clamp(cap, base_cap, debug.target_speed_preview);
    }

    double compute_track_margin_speed_cap(double effective_margin, double target_speed_preview) const {
        const double release_margin = std::max(gains_.track_margin_speed_release, 1e-3);
        if (effective_margin >= release_margin) {
            return std::numeric_limits<double>::infinity();
        }

        const double margin_ratio = std::clamp(effective_margin / release_margin, 0.0, 1.0);
        const double base_target = std::max(target_speed_preview, gains_.track_margin_speed_cap);
        return gains_.track_margin_speed_cap
             + (base_target - gains_.track_margin_speed_cap) * margin_ratio * margin_ratio;
    }

    std::pair<double, double> compute_track_margin_projection(const PIDDebugInfo& debug) const {
        const auto point = track_.get_point(debug.s);
        const double lateral_limit = (debug.n >= 0.0) ? point.width_left : point.width_right;
        const double margin = lateral_limit - std::abs(debug.n);
        const double lateral_velocity = debug.current_speed * std::sin(debug.d_heading);
        const double outward_rate = (std::abs(debug.n) > 1e-6)
            ? std::copysign(lateral_velocity, debug.n)
            : 0.0;
        const double projected_margin = margin
            - std::max(0.0, outward_rate) * std::max(gains_.track_margin_preview_time, 0.0);
        return {std::min(margin, projected_margin), outward_rate};
    }

    /// ステアリング制御（PID + FF）
    double compute_steering_control(
        const track::FrenetCoordinate& frenet,
        const vehicle::State14DOF& vehicle_state,
        double dt,
        const PlannedLineGeometry& line_geometry)
    {
        // 車両モデルの実装: 正のステアリング = 右旋回（時計回り）
        // ※ 回転行列 [cos,sin; -sin,cos] は時計回り回転（SAE標準とは逆）
        // 横偏差（nが目標より正：左側にずれている → 右に切る必要がある → 正の舵角）
        double e_lat = frenet.n - line_geometry.target_n;

        // 微分項（横偏差の変化率）
        double de_lat = (e_lat - state_.prev_lateral_error) / dt;

        // 積分項（アンチワインドアップ付き）
        state_.integral_lateral_error += e_lat * dt;
        state_.integral_lateral_error = std::clamp(
            state_.integral_lateral_error,
            -gains_.integral_limit_steer,
            gains_.integral_limit_steer
        );

        // トルクモード外側ループスケーリング
        // 内側ループの応答遅延により、高周波フィードバック項(Kh, Kff)を減衰
        double torque_scale = gains_.torque_mode ? gains_.torque_outer_loop_scale : 1.0;

        // PID項（正のn（左にずれ）→ 正の舵角 → 右旋回で中心線に戻る）
        double pid_term = gains_.Kp_steer * e_lat
                        + gains_.Kd_steer * de_lat
                        + gains_.Ki_steer * state_.integral_lateral_error;

        // ヘディング誤差項（正のd_heading → 正の舵角で補正）
        const double steer_preview = compute_effective_steering_preview_distance(vehicle_state.speed());
        last_debug_.steering_preview_distance_used = steer_preview;
        const double preview_s = steer_preview > 1e-6 ? wrap_track_s(frenet.s + steer_preview) : frenet.s;
        const auto preview_line_geometry = compute_tracking_line_geometry(preview_s);
        auto track_point = preview_line_geometry.center_point;
        const double map_theta = M_PI / 2.0 - vehicle_state.psi;
        const double heading_error = steer_preview > 1e-6
            ? normalize_angle(map_theta - preview_line_geometry.heading)
            : normalize_angle(map_theta - line_geometry.heading);
        double heading_term = torque_scale * gains_.Kh_steer * heading_error;

        // フィードフォワード項（左旋回 κ>0 → 負の舵角で左旋回）
        // ※ 車両モデルでは正のステアリング = 右旋回なので、左カーブには負の舵角が必要
        double ff_term = -torque_scale * gains_.Kff_steer * gains_.wheelbase
                       * preview_line_geometry.curvature;

        double pp_term = 0.0;
        if (gains_.Kpp_steer > 1e-9 && steer_preview > 1e-6) {
            const double map_x = vehicle_state.y;
            const double map_y = vehicle_state.x;
            const double target_x = track_point.x
                                  - std::sin(track_point.theta) * preview_line_geometry.target_n;
            const double target_y = track_point.y
                                  + std::cos(track_point.theta) * preview_line_geometry.target_n;
            const double dx = target_x - map_x;
            const double dy = target_y - map_y;
            const double alpha = normalize_angle(std::atan2(dy, dx) - map_theta);
            const double Ld = std::max(steer_preview, std::hypot(dx, dy));
            const double delta_left_positive = std::atan2(2.0 * gains_.wheelbase * std::sin(alpha), Ld);
            pp_term = -gains_.Kpp_steer * delta_left_positive;
        }

        // 合計舵角
        double delta = pid_term + heading_term + ff_term + pp_term;

        // リミット
        delta = std::clamp(delta, -gains_.max_steer, gains_.max_steer);

        // 状態更新
        state_.prev_lateral_error = e_lat;

        return delta;
    }

    double compute_progressive_throttle_limit(
        const track::FrenetCoordinate& frenet,
        double current_speed,
        double target_speed) const
    {
        if (!driver_max_throttle_unlimited() || gains_.max_throttle >= 0.999) {
            return gains_.max_throttle;
        }

        // max_throttle の完全撤廃は autocross の立ち上がり直後に破綻しやすかった。
        // そのため、preset の安全上限をベースにしつつ、
        // 進捗・追従誤差・曲率が十分に健全な直線区間でのみ 1.0 へ開放する。
        auto track_point = track_.get_point(frenet.s);

        double progress_gate = std::clamp(frenet.s / 35.0, 0.0, 1.0);
        double heading_gate = 1.0 - std::clamp(std::abs(frenet.d_heading) / 0.20, 0.0, 1.0);
        double lateral_gate = 1.0 - std::clamp(std::abs(frenet.n) / 0.75, 0.0, 1.0);
        double curvature_gate = 1.0 - std::clamp(std::abs(track_point.kappa) / 0.03, 0.0, 1.0);
        double overspeed = std::max(0.0, current_speed - target_speed);
        double speed_gate = 1.0 - std::clamp(overspeed / 1.0, 0.0, 1.0);

        double release = std::clamp(
            progress_gate * heading_gate * lateral_gate * curvature_gate * speed_gate,
            0.0,
            1.0
        );

        return gains_.max_throttle + (1.0 - gains_.max_throttle) * release;
    }

    /// 速度制御（PI + ブレーキ）
    std::pair<double, double> compute_speed_control(
        double current_speed,
        double target_speed,
        double dt)
    {
        // 速度誤差
        double e_speed = target_speed - current_speed;

        // 積分項（アンチワインドアップ付き）
        state_.integral_speed_error += e_speed * dt;
        state_.integral_speed_error = std::clamp(
            state_.integral_speed_error,
            -gains_.integral_limit_speed,
            gains_.integral_limit_speed
        );

        // PI制御（スロットル側）
        double control = gains_.Kp_speed * e_speed + gains_.Ki_speed * state_.integral_speed_error;

        // スロットル/ブレーキに分離
        // ブレーキはbrake_gain_ratioで感度を下げる:
        //   高Kp_speedでも急ブレーキを防ぎ、コーナリング中の横グリップ確保
        double throttle = 0.0;
        double brake = 0.0;

        const double throttle_limit = driver_max_throttle_unlimited() ? 1.0 : gains_.max_throttle;

        if (control > 0.0) {
            // 加速: Kp_speedをフルに適用
            throttle = std::min(control, throttle_limit);
        } else {
            // 減速: brake_gain_ratioで緩和（デフォルト35%）
            brake = std::min(-control * gains_.brake_gain_ratio, gains_.max_brake);
        }

        return {throttle, brake};
    }



    int select_gear(double speed) const {
        return select_gear_from_vehicle_params(speed, vehicle_params_);
    }

    void rebuild_planned_line_geometry_cache() {
        planned_line_geometry_profile_.clear();
        if (!gains_.line_planner.enabled) {
            return;
        }

        const double length = track_.length();
        if (length <= 1e-6) {
            return;
        }

        constexpr double kGeometryCacheStep = 0.25;  // [m], line-offset transitions are sharp near autocross gates.
        const double step = kGeometryCacheStep;
        for (double s = 0.0; s < length; s += step) {
            planned_line_geometry_profile_.push_back(
                compute_planned_line_geometry(track_, s, gains_.line_planner));
        }
        planned_line_geometry_profile_.push_back(
            compute_planned_line_geometry(track_, length, gains_.line_planner));
    }

    PlannedLineGeometry lookup_planned_line_geometry(double s) const {
        const double wrapped_s = wrap_track_s(s);
        if (planned_line_geometry_profile_.size() < 2U) {
            return compute_planned_line_geometry(track_, wrapped_s, gains_.line_planner);
        }

        auto upper = std::lower_bound(
            planned_line_geometry_profile_.begin(),
            planned_line_geometry_profile_.end(),
            wrapped_s,
            [](const PlannedLineGeometry& geometry, double value) {
                return geometry.s < value;
            });

        if (upper == planned_line_geometry_profile_.begin()) {
            return *upper;
        }
        if (upper == planned_line_geometry_profile_.end()) {
            return planned_line_geometry_profile_.back();
        }

        const auto& lo = *(upper - 1);
        const auto& hi = *upper;
        const double ds = std::max(hi.s - lo.s, 1e-9);
        const double alpha = std::clamp((wrapped_s - lo.s) / ds, 0.0, 1.0);
        auto lerp = [alpha](double a, double b) {
            return a + (b - a) * alpha;
        };
        auto lerp_angle = [alpha](double a, double b) {
            return a + normalize_angle(b - a) * alpha;
        };

        PlannedLineGeometry geometry;
        geometry.s = wrapped_s;
        geometry.center_point = track_.get_point(wrapped_s);
        geometry.target_n = lerp(lo.target_n, hi.target_n);
        geometry.heading = lerp_angle(lo.heading, hi.heading);
        geometry.heading_offset = lerp_angle(lo.heading_offset, hi.heading_offset);
        geometry.curvature = lerp(lo.curvature, hi.curvature);
        geometry.dn_ds = lerp(lo.dn_ds, hi.dn_ds);
        geometry.ddn_ds2 = lerp(lo.ddn_ds2, hi.ddn_ds2);
        geometry.centerline_curvature = geometry.center_point.kappa;
        geometry.plan.target_n = lerp(lo.plan.target_n, hi.plan.target_n);
        geometry.plan.curvature_now = lerp(lo.plan.curvature_now, hi.plan.curvature_now);
        geometry.plan.curvature_ahead = lerp(lo.plan.curvature_ahead, hi.plan.curvature_ahead);
        geometry.plan.curvature_behind = lerp(lo.plan.curvature_behind, hi.plan.curvature_behind);
        geometry.plan.usable_half_width = lerp(lo.plan.usable_half_width, hi.plan.usable_half_width);
        return geometry;
    }

    PlannedLineGeometry compute_tracking_line_geometry(double s) const {
        if (gains_.line_planner.track_geometry) {
            return lookup_planned_line_geometry(s);
        }
        if (planned_line_geometry_profile_.size() >= 2U) {
            auto geometry = lookup_planned_line_geometry(s);
            geometry.target_n = geometry.plan.target_n;
            geometry.heading = geometry.center_point.theta;
            geometry.heading_offset = 0.0;
            geometry.curvature = geometry.center_point.kappa;
            return geometry;
        }

        PlannedLineGeometry geometry;
        const double wrapped_s = wrap_track_s(s);
        geometry.s = wrapped_s;
        geometry.center_point = track_.get_point(wrapped_s);
        geometry.centerline_curvature = geometry.center_point.kappa;
        geometry.plan = compute_boundary_line_plan(track_, wrapped_s, gains_.line_planner);
        geometry.target_n = geometry.plan.target_n;
        geometry.heading = geometry.center_point.theta;
        geometry.heading_offset = 0.0;
        geometry.curvature = geometry.center_point.kappa;
        return geometry;
    }

    void sync_steering_contract() {
        if (!vehicle_params_.steering.torque_mode) {
            return;
        }
        gains_.torque_mode = true;
        gains_.R_steer           = vehicle_params_.steering.R_steer;
        gains_.max_steer_torque  = vehicle_params_.steering.T_driver_max;

        // A-1: ω_n / ζ ベース自動ゲイン正規化
        // I_steer や C_steer_torque が変わっても固有振動数・減衰が一定に保たれる。
        // 手動チューンを保持したい場合は inner_loop_auto_tune = false で無効化できる。
        if (gains_.inner_loop_auto_tune) {
            const double I = std::max(vehicle_params_.steering.I_steer, 1e-6);
            const double C = std::max(vehicle_params_.steering.C_steer_torque, 0.0);
            const double omega_n = gains_.inner_loop_omega_n;  // 目標固有振動数 [rad/s]
            const double zeta    = gains_.inner_loop_zeta;     // 目標減衰係数 [-]

            // Kp = ω_n² * I
            gains_.Kp_steer_inner = omega_n * omega_n * I;

            // Kd = 2*ζ*ω_n*I - C  (減衰係数から粘性ダンパー分を引く)
            // 負にならないよう下限を 0 に制限
            gains_.Kd_steer_inner = std::max(2.0 * zeta * omega_n * I - C, 0.0);

            // Ki は既定では 10% of Kp。デバッグ時は auto_tune を維持したまま Ki だけ上書き可能。
            gains_.Ki_steer_inner = gains_.inner_loop_ki_override >= 0.0
                ? gains_.inner_loop_ki_override
                : 0.10 * gains_.Kp_steer_inner;

            // 最大ドライバートルクで Ki アンチワインドアップを自動スケール
            // inner_integral_ のクランプ範囲は ±(T_driver_max / Ki_steer_inner) の 20% 相当
            // 実際のクランプは compute_control 内の ±0.5 rad で行う（既存）
        }
    }

    static double normalize_angle(double angle) {
        if (std::isnan(angle) || std::isinf(angle)) return 0.0;
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0.0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    const track::CurvatureBasedTrack& track_;
    const std::vector<laptime::VelocityPoint>& velocity_profile_;
    PIDGains gains_;
    vehicle::VehicleParams vehicle_params_;
    track::FrenetConverter converter_;
    PIDState state_;
    SteeringActuatorState steering_actuator_state_;
    PIDDebugInfo last_debug_;
    std::vector<PlannedLineGeometry> planned_line_geometry_profile_;
};

/// プリセットゲイン：保守的（安定重視）
inline PIDGains conservative_gains() {
    PIDGains gains;
    gains.Kp_steer = 0.5;
    gains.Kd_steer = 0.2;
    gains.Ki_steer = 0.02;
    gains.Kh_steer = 0.3;
    gains.Kff_steer = 0.8;
    gains.Kp_speed = 0.1;
    gains.Ki_speed = 0.02;
    gains.max_throttle = 0.3;
    return gains;
}

/// プリセットゲイン：アグレッシブ（速度重視）
inline PIDGains aggressive_gains() {
    PIDGains gains;
    gains.Kp_steer = 1.2;
    gains.Kd_steer = 0.5;
    gains.Ki_steer = 0.1;
    gains.Kh_steer = 0.8;
    gains.Kff_steer = 1.2;
    gains.Kp_speed = 0.2;
    gains.Ki_speed = 0.05;
    gains.max_throttle = 1.0;
    return gains;
}

/// プリセットゲイン：最適化済み（figure_eightトラック用）
/// ベイズ最適化により得られた最適ゲイン（2026-02-11、半径25m）
inline PIDGains optimized_figure_eight_gains() {
    PIDGains gains;
    gains.Kp_steer = 1.992569;
    gains.Kd_steer = 0.057854;
    gains.Ki_steer = 0.064521;
    gains.Kh_steer = 0.923226;
    gains.Kff_steer = 0.588280;
    gains.Kp_speed = 0.767312;
    gains.Ki_speed = 0.111730;
    gains.max_throttle = 1.0;
    return gains;
}

/// プリセットゲイン：最適化済み（circleトラック用）
/// グリッドサーチにより得られた最適ゲイン（2026-02-12、半径30m）
/// RMS横偏差 0.8cm、最大横偏差 1.0cm を達成
inline PIDGains optimized_circle_gains() {
    PIDGains gains;
    gains.Kp_steer = 1.2;
    gains.Kd_steer = 0.2;
    gains.Ki_steer = 0.05;
    gains.Kh_steer = 0.3;
    gains.Kff_steer = 1.0;
    gains.Kp_speed = 0.3;
    gains.Ki_speed = 0.05;
    gains.max_throttle = 0.5;
    return gains;
}

/// プリセットゲイン：最適化済み（skidpadトラック用）
/// SAEスキッドパッド（半径9.125m）向けの初期ゲイン
/// 小半径トラックのためFFゲインが重要
inline PIDGains optimized_skidpad_gains() {
    PIDGains gains;
    gains.Kp_steer = 1.5;
    gains.Kd_steer = 0.15;
    gains.Ki_steer = 0.05;
    gains.Kh_steer = 0.6;
    gains.Kff_steer = 1.0;    // フィードフォワードが重要（小半径）
    gains.Kp_speed = 0.5;
    gains.Ki_speed = 0.1;
    gains.brake_gain_ratio = 0.50;
    gains.corner_throttle_reduction = 0.95;  // タイトコーナー(R=9m)での過ステア防止
    gains.max_throttle = 0.80;
    return gains;
}


/// プリセットゲイン：最適化済み（ASE Autocross / closed-loop CSV 用）
/// 2026-04-02 改訂:
///   preview ベースの目標速度抑制を前提に、S 字入口の過大進入を減らしつつ
///   lap time を落としすぎない方向で再調整した。
/// 改訂ポイント:
///   Kh_steer  0.60 → 0.70  (曲率反転区間の heading correction を少し強める)
///   Kff_steer 1.20 → 1.30  (preview で減速した後の先行操舵を補強する)
///   Kp_speed  0.40 → 0.20  (速度追従を緩め、速度プロファイル追い越しを抑える)
///   Ki_speed  0.01 → 0.00  (S 字入口での speed integrator 残留を避ける)
///   target_speed_preview_distance = 4.0m  (未来窓の最小 target speed を使用)
inline PIDGains optimized_autocross_gains() {
    PIDGains gains;
    gains.Kp_steer = 0.735;
    gains.Kd_steer = 0.357;
    gains.Ki_steer = 0.027;
    gains.Kh_steer = 0.669;
    gains.Kff_steer = 2.20;
    gains.Kp_speed = 0.10;
    gains.Ki_speed = 0.00;
    gains.brake_gain_ratio = 0.50;
    gains.corner_throttle_reduction = 0.61;
    gains.max_throttle = 0.45;
    gains.max_steer = 0.55;
    gains.target_speed_preview_distance = 14.0;
    gains.steering_preview_distance = 4.0;
    gains.Kpp_steer = 0.0;
    return gains;
}

/// プリセットゲイン：ASE Autocross の高速車両向け
/// optimized_autocross.json のような高μ・軽量側の車両で使う攻めた設定。
/// デフォルト車両には進入速度が高すぎるため明示プリセットに分離する。
inline PIDGains optimized_autocross_fast_gains() {
    PIDGains gains = optimized_autocross_gains();
    gains.Kp_speed = 0.15;
    gains.brake_gain_ratio = 0.35;
    gains.corner_throttle_reduction = 0.80;
    gains.max_throttle = 0.60;
    gains.target_speed_preview_distance = 8.0;
    gains.steering_preview_distance = 4.0;
    return gains;
}

/// プリセットゲイン：ASE Autocross のトルク操舵向け
/// 角度モード用の強い先読みFFはトルク操舵では遅れと飽和を誘発しやすいため、
/// ステア遅れ・飽和・フレーム捻りに応じた動的速度制限を前提にする。
inline PIDGains optimized_autocross_torque_gains() {
    PIDGains gains;
    gains.torque_mode = true;
    gains.Kp_steer = 0.70;
    gains.Kd_steer = 0.25;
    gains.Ki_steer = 0.03;
    gains.Kh_steer = 0.70;
    gains.Kff_steer = 1.60;
    gains.Kp_speed = 0.20;
    gains.Ki_speed = 0.00;
    gains.brake_gain_ratio = 0.80;
    // Torque steering already uses steering-lag and track-margin speed targets.
    // Keep the post-PI throttle limiter mild so it does not mask those predictors
    // and delay braking into tight autocross features.
    gains.corner_throttle_reduction = 0.10;
    gains.max_throttle = 0.50;
    gains.target_speed_preview_distance = 6.0;
    gains.steering_preview_distance = 10.0;
    gains.Kpp_steer = 1.0;
    gains.steering_preview_authority_threshold = 0.90;
    gains.torque_target_speed_scale = 0.90;
    gains.torque_outer_loop_scale = 0.80;
    gains.max_tire_steer_rate = 3.5;
    gains.torque_lag_lead_gain = 0.0;
    gains.torque_lag_lead_time = 0.0;
    gains.torque_lag_lead_max_tire_angle = 0.06;
    gains.torque_speed_recovery_gain = 0.70;
    gains.torque_speed_recovery_curvature = 0.012;
    gains.torque_speed_recovery_lag_ratio = 0.12;
    gains.torque_speed_recovery_lateral_error = 0.50;
    gains.torque_speed_recovery_heading_error = 0.12;
    gains.torque_speed_recovery_preview_gap = 0.08;
    gains.torque_speed_recovery_reach_time = 0.04;
    gains.torque_reach_speed_cap = 10.0;
    gains.torque_reach_time_start = 0.13;
    gains.torque_reach_time_full = 0.22;
    gains.torque_reach_outward_rate_start = 1.2;
    gains.torque_reach_outward_rate_full = 3.0;
    gains.torque_reach_min_preview_gap = 0.18;
    gains.inner_loop_omega_n = 80.0;
    gains.inner_loop_zeta = 0.70;
    gains.torque_lag_speed_penalty = 0.35;
    gains.torque_saturation_speed_penalty = 0.15;
    gains.chassis_twist_speed_penalty = 0.10;
    gains.track_margin_preview_time = 0.50;
    gains.track_margin_speed_release = 1.0;
    gains.torque_stability_yaw_threshold = 0.30;
    gains.torque_stability_yaw_gain = 0.30;
    gains.torque_stability_slip_threshold = 0.12;
    gains.torque_stability_slip_gain = 1.20;
    gains.line_planner.enabled = true;
    gains.line_planner.offset_fraction = 0.05;
    gains.line_planner.safety_margin = 0.55;
    gains.line_planner.preview_distance = 10.0;
    return gains;
}

} // namespace fsae::driver
