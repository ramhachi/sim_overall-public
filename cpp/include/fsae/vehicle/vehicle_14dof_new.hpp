
#pragma once
/// @file vehicle_14dof_new.hpp
/// @brief 14DOF Full Vehicle Model
///
/// 設計書に基づく完全な14自由度車両モデル
///
/// 自由度構成:
///   - 6 DOF ばね上質量 (x, y, z, φ, θ, ψ)
///   - 4 DOF ばね下垂直 (z_u × 4)
///   - 4 DOF 車輪回転 (ω × 4)
///
/// 統合サブシステム:
///   - TMeasyタイヤ（緩和長モデル）
///   - Bouc-Wenダンパー（ヒステリシス）
///   - LuGre摩擦（スティック・スリップ）
///   - アンチロールバー（前後）
///   - バンプストップ
///   - 空力（ダウンフォース/ドラッグ）
///
/// 状態変数: 44
///   [x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r,    (12)
///    z_u(4), vz_u(4),                           (8)
///    ω(4),                                      (4)
///    tire_xi(4), tire_eta(4),                   (8)
///    damper_z(4), friction_z(4)]                (8)

#include "fsae/vehicle/body_moments.hpp"
#include <fsae/vehicle/vehicle_params.hpp>
#include <fsae/vehicle/vehicle_state.hpp>
#include <fsae/vehicle/drs_system.hpp>
#include <fsae/vehicle/steering_torque.hpp>
#include <fsae/physics/tire/tmeasy.hpp>
#include <fsae/physics/tire/nn_lookup_tire.hpp>
#include <fsae/physics/tire/magic_formula.hpp>
#include <fsae/physics/aero/aero_map_4d.hpp>
#include <fsae/physics/tire/tire_thermal_integrator.hpp>
#include <fsae/physics/suspension/bouc_wen.hpp>
#include <fsae/physics/suspension/lugre.hpp>
#include <fsae/physics/powertrain/engine.hpp>
#include <fsae/physics/powertrain/lsd.hpp>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace fsae::vehicle {

/// 14DOF車両状態
struct State14DOF {
    // ======== ばね上位置（慣性系）========
    double x = 0, y = 0, z = 0;           // 位置 [m]
    double phi = 0, theta = 0, psi = 0;   // オイラー角 [rad]
    
    // ======== ばね上速度（車体系）========
    double vx = 0, vy = 0, vz = 0;        // 並進速度 [m/s]
    double p = 0, q = 0, r = 0;           // 角速度 [rad/s]
    
    // ======== ばね下状態 ========
    std::array<double, 4> z_u = {0, 0, 0, 0};     // 垂直位置 [m]
    std::array<double, 4> vz_u = {0, 0, 0, 0};    // 垂直速度 [m/s]
    
    // ======== 車輪回転 ========
    std::array<double, 4> omega = {0, 0, 0, 0};   // 角速度 [rad/s]
    
    // ======== タイヤ変形・遅れ状態（モデル分岐で意味と単位が変化）========
    //   TMeasy : tire_xi = 縦変形 [m], tire_eta = 横変形 [m]
    //   MF     : tire_xi = 縦スリップ(遅れ) [-], tire_eta = 横スリップ角(遅れ) [rad]
    //   PINN   : tire_xi = 縦スリップ(遅れ) [-], tire_eta = 横スリップ角(遅れ) [deg]
    std::array<double, 4> tire_xi = {0, 0, 0, 0};
    std::array<double, 4> tire_eta = {0, 0, 0, 0};
    
    // ======== Bouc-Wen履歴変数 ========
    std::array<double, 4> damper_z = {0, 0, 0, 0};
    
    // ======== LuGre毛変形 ========
    std::array<double, 4> friction_z = {0, 0, 0, 0};

    // ======== フレームねじり（Phase A1）========
    double phi_twist = 0;        // フレームねじり角 [rad]
    double dphi_twist = 0;       // フレームねじり角速度 [rad/s]

    // ======== ステアリングホイール（Phase A2）========
    double delta_sw = 0;         // ステアリングホイール角 [rad]
    double ddelta_sw = 0;        // ステアリングホイール角速度 [rad/s]

    static constexpr int SIZE = 44;  // 42 → 44

    // 便利なアクセサ
    double speed() const { return std::sqrt(vx*vx + vy*vy); }
    double sideslip() const { return std::atan2(vy, std::max(vx, 0.1)); }
};

/// 14DOF車両モデル
class Vehicle14DOF_New {
public:
    using Params = VehicleParams;
    using StateVector = Eigen::Matrix<double, State14DOF::SIZE, 1>;
    static constexpr int FL = 0, FR = 1, RL = 2, RR = 3;
    
    explicit Vehicle14DOF_New(const Params& params) : params_(params) {
        if (!std::isfinite(params_.tire.pinn_eval_rate_hz) ||
            params_.tire.pinn_eval_rate_hz < 0.0) {
            throw std::invalid_argument("tire.pinn_eval_rate_hz must be finite and non-negative");
        }
        wheel_positions_ = build_wheel_positions();
        setup_tire_params();
        setup_bouc_wen_params();
        setup_lugre_params();
        setup_powertrain();
        setup_aero_map();
        compute_equilibrium();
        initialize_selected_tire_model();

        reset();  // 初期キャッシュ更新
    }
    
    // ======== アクセサ ========
    const State14DOF& state() const { return state_; }
    State14DOF& state() { return state_; }
    const Params& params() const { return params_; }
    double time() const { return time_; }
    double z_equilibrium() const { return z_equilibrium_; }

    // DRS アクセサ
    DRSSystem& drs_system() { return drs_system_; }
    const DRSSystem& drs_system() const { return drs_system_; }
    bool is_drs_active() const { return drs_active_; }
    const std::array<double, 4>& tire_Mz() const { return tire_Mz_; }
    /// 前輪キングピン軸トルク [FL, FR] [Nm]（角度モードでも診断用に更新, #5）
    const std::array<double, 2>& get_kingpin_torque() const { return kingpin_torque_; }
    
    /// 路面高さ [FL,FR,RL,RR] [m]（Z 下向き正: 路面が上がると負, #14）。step 間は保持。
    void set_road_input(const std::array<double, 4>& z_road) { z_road_ = z_road; }
    const std::array<double, 4>& road_input() const { return z_road_; }

    // ======== 初期化 ========
    void set_initial_velocity(double vx) {
        apply_aero_static_equilibrium(vx);
        state_.vx = vx;
        for (int i = 0; i < 4; ++i) {
            state_.omega[i] = vx / compute_effective_rolling_radius(static_Fz_[i] + params_.mass.m_unsprung[i] * 9.81);
        }
        invalidate_pinn_force_cache();
        capture_pinn_cache_state_snapshot();
    }
    
    void reset() {
        state_ = State14DOF{};
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
        }
        time_ = 0;
        brake_pressure_lagged_ = 0.0;
        last_gear_ = -1;
        shift_end_time_ = -1.0;
        invalidate_pinn_force_cache();
        
        // 初期状態でキャッシュを更新
        DriverInput dummy_input{};
        compute_derivatives(state_, dummy_input, true);
        capture_pinn_cache_state_snapshot();
    }
    
    // ======== シミュレーションステップ ========
    void step(double dt, const DriverInput& input) {
        const double drs_steering_angle = compute_driver_tire_steer_angle(state_, input);

        // DRS状態更新（安全性チェック含む）— RK4積分の前に実行
        drs_active_ = drs_system_.update(
            input.drs_button,
            state_.speed(),
            state_.vx,
            state_.r,
            drs_steering_angle,
            dt);

        if (last_gear_ >= 0 && input.gear != last_gear_) {
            shift_end_time_ = time_ + params_.powertrain.shift_time_s;
        }
        last_gear_ = input.gear;

        if (params_.brake.pressure_lag_s > 0.0) {
            // ponytail: lag held over the RK4 step (ZOH); make it a state if tau ~ dt.
            brake_pressure_lagged_ += (1.0 - std::exp(-dt / params_.brake.pressure_lag_s))
                * (compute_brake_pressure_cmd(input.brake) - brake_pressure_lagged_);
        }

        StateVector x = state_to_vector(state_);
        if (pinn_cache_state_snapshot_valid_ &&
            (x.array() != pinn_cache_state_snapshot_.array()).any()) {
            // state() intentionally exposes a mutable reference for legacy
            // callers. Detect external edits before reusing held tire forces.
            invalidate_pinn_force_cache();
        }
        
        auto f = [this, &input](const StateVector& x_) {
            return compute_derivatives(vector_to_state(x_), input);
        };
        
        // RK4積分
        StateVector k1 = f(x);
        StateVector k2 = f(x + 0.5 * dt * k1);
        StateVector k3 = f(x + 0.5 * dt * k2);
        StateVector k4 = f(x + dt * k3);
        
        x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        state_ = vector_to_state(x);
        time_ += dt;

        // タイヤ変形量の飽和（RK4積分後：xi/eta の無制限成長を防ぐ）
        constexpr double TIRE_XI_MAX = 1.0;
        const double tire_eta_max = nn_model_loaded_ ? 30.0 : 1.0;
        for (int i = 0; i < 4; ++i) {
            state_.tire_xi[i]  = std::clamp(state_.tire_xi[i],  -TIRE_XI_MAX, TIRE_XI_MAX);
            state_.tire_eta[i] = std::clamp(state_.tire_eta[i], -tire_eta_max, tire_eta_max);
        }

        compute_derivatives(state_, input, true);
        
        // タイヤ温度更新（NNモデル使用時のみ）
        if (nn_model_loaded_) {
            std::array<physics::tire::TireForces, 4> tire_forces;
            std::array<physics::tire::TireSlipInfo, 4> slip_info;
            
            const auto& wheel_pos = get_wheel_positions();
            const double delta_driver = compute_driver_tire_steer_angle(state_, input);
            const std::array<double, 4> steer = {delta_driver, delta_driver, 0, 0};
            
            for (int i = 0; i < 4; ++i) {
                tire_forces[i].Fx = tire_Fx_[i];
                tire_forces[i].Fy = tire_Fy_[i];
                tire_forces[i].Mx = 0.0;
                tire_forces[i].Mz = 0.0;
                
                const Eigen::Vector3d contact_position(
                    wheel_pos[i].x(), wheel_pos[i].y(), params_.mass.h_cg);
                const Eigen::Vector3d contact_velocity = compute_contact_patch_velocity(
                    Eigen::Vector3d(state_.vx, state_.vy, state_.vz),
                    Eigen::Vector3d(state_.p, state_.q, state_.r),
                    contact_position);
                double vx_contact = contact_velocity.x();
                double vy_contact = contact_velocity.y();
                
                double c = std::cos(steer[i]);
                double s = std::sin(steer[i]);
                double vx_wheel = c * vx_contact + s * vy_contact;
                double vy_wheel = -s * vx_contact + c * vy_contact;
                double omega_r = state_.omega[i] * compute_effective_rolling_radius(tire_Fz_[i]);
                
                slip_info[i].vsx = vx_wheel - omega_r;
                slip_info[i].vsy = vy_wheel;
            }
            
            double v_wind = state_.speed();
            double T_ambient = params_.tire.ambient_temperature_c;
            
            tire_thermal_.update(dt, tire_forces, slip_info, v_wind, T_ambient);
        }
        capture_pinn_cache_state_snapshot();
    }

    // ======== 適応的タイムステップ（ガイド §12.2, §12.6） ========
    /// @brief バンプストップ接触やLuGre高剛性時に自動的にdt短縮
    /// @param dt 基本タイムステップ [s] (通常: 0.001)
    /// @param input ドライバー入力
    void step_adaptive(double dt, const DriverInput& input) {
        // バンプストップ接触判定
        bool bumpstop_contact = false;
        const auto& sp = params_.suspension;
        const double bump_threshold = 0.9 * std::min(sp.bump_stop_gap,
            sp.bump_stop_gap_rebound < 0.0 ? sp.bump_stop_gap : sp.bump_stop_gap_rebound);

        for (int i = 0; i < 4; ++i) {
            const double x = state_.z - state_.z_u[i] - (z_equilibrium_ - z_u_equilibrium_[i]);
            // Gaps live in element coordinates when MR applies to the bump stop (#10).
            if (std::abs(damper_motion_ratio(i, x) * x) > bump_threshold) {
                bumpstop_contact = true;
                break;
            }
        }

        // LuGre高剛性判定
        bool lugre_stiff = (params_.suspension.lugre_sigma0 > 2000.0);

        // 適応的タイムステップ決定
        int substeps = 1;
        if (bumpstop_contact) {
            // バンプストップ接触: dt を 1/5 に短縮
            substeps = 5;
        } else if (lugre_stiff) {
            // LuGre高剛性: dt を 1/2 に短縮
            substeps = 2;
        }
        // Explicit substep override (#7).
        substeps = std::max(substeps, std::max(1, sp.lugre_substeps));
        const double dt_effective = dt / substeps;

        // サブステップ積分
        for (int i = 0; i < substeps; ++i) {
            step(dt_effective, input);
        }
    }
    
    // ======== デバッグ用 ========
    /// Common low-speed slip definition shared by PINN/MF/TMeasy (#20):
    ///   V   = sqrt(vx^2 + v_N^2)
    ///   kappa = (omega*Re - vx) / V,  alpha = -atan2(vy, V)
    struct CommonSlip {
        double transport;  // [m/s]
        double kappa;      // [-]
        double alpha_rad;  // [rad]
    };
    CommonSlip common_slip(double vx_wheel, double vy_wheel, double omega_r) const {
        const double v_n = std::max(params_.tire.v_N, 1e-6);
        const double transport = std::sqrt(vx_wheel * vx_wheel + v_n * v_n);
        return {transport, (omega_r - vx_wheel) / transport,
                -std::atan2(vy_wheel, transport)};
    }
    std::array<double, 4> get_tire_Fz() const { return tire_Fz_; }
    std::array<double, 4> get_tire_Fx() const { return tire_Fx_; }
    std::array<double, 4> get_tire_Fy() const { return tire_Fy_; }
    std::array<double, 4> get_tire_Fx_body() const { return tire_Fx_body_; }
    std::array<double, 4> get_tire_Fy_body() const { return tire_Fy_body_; }
    std::array<double, 4> get_tire_Fz_body() const { return tire_Fz_body_; }
    std::array<double, 4> get_tire_Mz() const { return tire_Mz_; }
    bool is_nn_tire_loaded() const { return nn_model_loaded_; }
    bool is_magic_formula_tire_loaded() const { return mf_model_loaded_; }
    void set_nn_tire_runtime_diagnostics_enabled(bool enabled) {
        nn_tire_model_.set_runtime_diagnostics_enabled(enabled);
    }
    void reset_nn_tire_runtime_diagnostics() const {
        nn_tire_model_.reset_runtime_diagnostics();
    }
    physics::tire::NNRuntimeDiagnostics nn_tire_runtime_diagnostics() const {
        return nn_tire_model_.runtime_diagnostics();
    }
    std::string active_tire_runtime_description() const {
        if (nn_model_loaded_) {
            return nn_tire_model_.runtime_description();
        }
        if (mf_model_loaded_) {
            return "magic_formula";
        }
        return "tmeasy";
    }
    std::array<double, 4> get_bw_forces() const { return bw_forces_; }
    std::array<double, 4> get_lugre_forces() const { return lugre_forces_; }
    std::array<double, 4> get_spring_forces() const { return spring_forces_; }
    std::array<double, 4> get_pou_displacement() const { return pou_displacement_; }
    std::array<double, 4> get_pou_camber() const { return pou_camber_; }

    // Phase A1: フレーム柔軟性アクセサ
    double get_frame_twist() const { return state_.phi_twist; }
    double get_frame_twist_rate() const { return state_.dphi_twist; }
    std::pair<double, double> get_split_roll_angles() const {
        return compute_split_roll_angles(state_.phi, state_.phi_twist);
    }

    // Phase A2: ステアリングトルクアクセサ
    double get_steering_wheel_angle() const { return state_.delta_sw; }
    double get_steering_wheel_rate() const { return state_.ddelta_sw; }
    double get_tire_angle() const {
        if (params_.steering.torque_mode) {
            return steering_wheel_to_tire_angle(state_.delta_sw, params_.steering.R_steer);
        } else {
            // 角度モードでは直接入力値（状態には保存されていない）
            return 0.0;  // または最後の入力値をキャッシュする必要がある
        }
    }

    /// POU（Pushrod on Upright）によるジャッキアップ変位を計算（テスト用にpublic）
    /// @param wheel_index 車輪インデックス
    /// @param steer_angle 各輪のステアリング角 [rad]
    /// @return POU変位 [m] (正: スプリング圧縮方向)
    double compute_pou_displacement(int wheel_index, double steer_angle) const {
        return compute_pou_displacement(wheel_index, 0.0, steer_angle);
    }

    /// Closed-chain jack (damper compression) from the POU LUT (#44).
    double compute_pou_displacement(int wheel_index, double travel, double steer_angle) const {
        const auto& lut = params_.suspension.pou_lut[wheel_index];
        const double c1 = params_.suspension.pou_jack_c1[wheel_index];
        const double c2 = params_.suspension.pou_jack_c2[wheel_index];
        const double poly = c1 * steer_angle + c2 * steer_angle * steer_angle;
        if (lut.valid() && lut.jack_m.size() == lut.travel_m.size()) {
            const double v = interp2_clamped(lut.steer_rad, lut.travel_m, lut.jack_m,
                                             steer_angle, travel, poly);
            if (std::isfinite(v)) return v;
        }
        return poly;
    }

    /// POU motion ratio d(damper)/d(wheel travel) from the LUT (#44).
    double pou_mr_at(int wheel_index, double travel, double steer_angle) const {
        const auto& lut = params_.suspension.pou_lut[wheel_index];
        if (lut.valid() && lut.mr.size() == lut.travel_m.size()) {
            const double v = interp2_clamped(lut.steer_rad, lut.travel_m, lut.mr,
                                             steer_angle, travel, motion_ratio_at(wheel_index, travel));
            if (std::isfinite(v)) return v;
        }
        return motion_ratio_at(wheel_index, travel);
    }

    double pou_damper_mr_at(int wheel_index, double travel, double steer_angle) const {
        return params_.suspension.mr_applies_to_dampers
            ? pou_mr_at(wheel_index, travel, steer_angle) : 1.0;
    }

    double eval_quadratic(const std::array<double, 3>& coeffs, double x) const {
        return coeffs[0] + coeffs[1] * x + coeffs[2] * x * x;
    }

    bool is_zero_polynomial(const std::array<double, 3>& coeffs) const {
        return std::abs(coeffs[0]) < 1e-12
            && std::abs(coeffs[1]) < 1e-12
            && std::abs(coeffs[2]) < 1e-12;
    }

    /// サスペンションストロークとステアリング角からキャンバー角を計算（テスト用にpublic）
    /// @param wheel_index 車輪インデックス (0=FL, 1=FR, 2=RL, 3=RR)
    /// @param susp_travel_from_eq 平衡位置からのサスペンションストローク [m]
    /// @param steer_angle 各輪のステアリング角 [rad]
    /// @return キャンバー角 [rad] (負: ネガティブキャンバー)
    double compute_camber_angle(
        int wheel_index,
        double susp_travel_from_eq,
        double steer_angle,
        double axle_roll_angle,
        const std::array<double, 6>& kc_loads) const {
        const auto& kc = params_.suspension.kc_corners[wheel_index];
        const auto& sv = kc.samples;
        const bool has_travel_lut = kc.valid && sv.travel_m.size() >= 2
            && sv.camber_rad.size() == sv.travel_m.size();
        const bool has_steer_lut = kc.valid && sv.steer_rad.size() >= 2
            && sv.steer_camber_rad.size() == sv.steer_rad.size();
        const bool has_roll_lut = kc.valid && sv.roll_rad.size() >= 2
            && sv.roll_camber_residual_rad.size() == sv.roll_rad.size();
        const bool kc_owns_steer_camber = kc.valid
            && (has_steer_lut || !is_zero_polynomial(kc.kinematics.camber_vs_steer));
        double camber = params_.suspension.static_camber[wheel_index];
        if (kc.valid) {
            if (has_travel_lut) {
                camber = lut_eval(sv.travel_m, sv.camber_rad, susp_travel_from_eq, camber);
            } else if (is_zero_polynomial(kc.kinematics.camber_vs_travel)) {
                camber += params_.suspension.camber_gain[wheel_index] * susp_travel_from_eq;
            } else {
                camber = eval_quadratic(kc.kinematics.camber_vs_travel, susp_travel_from_eq);
            }
            if (has_steer_lut) {
                camber += lut_eval(sv.steer_rad, sv.steer_camber_rad, steer_angle, 0.0);
            } else if (kc_owns_steer_camber) {
                camber += eval_quadratic(kc.kinematics.camber_vs_steer, steer_angle);
            }
            // camber_vs_travel already sees the roll-induced +-t/2*phi travel, so the
            // roll term is applied only without it (#38). Dense roll residuals are
            // zero-referenced to avoid a static offset.
            if (has_roll_lut) {
                camber += lut_eval(sv.roll_rad, sv.roll_camber_residual_rad, axle_roll_angle, 0.0)
                        - lut_eval(sv.roll_rad, sv.roll_camber_residual_rad, 0.0, 0.0);
            } else if (!has_travel_lut && is_zero_polynomial(kc.kinematics.camber_vs_travel)) {
                camber += kc.kinematics.roll_camber_gain * axle_roll_angle;
            }

            for (int i = 0; i < 6; ++i) {
                camber += kc.compliance.camber_per_load[i] * kc_loads[i];
            }
        } else {
            camber += params_.suspension.camber_gain[wheel_index] * susp_travel_from_eq;
        }

        // 2. POU: ステアリング誘発キャンバー (Δγ = c1·δ + c2·δ²)
        // ジオメトリ由来の camber_vs_steer がある場合は二重計上を避けるためPOU側を適用しない
        const bool has_pou_lut = params_.suspension.pou_lut[wheel_index].valid();
        if (!kc_owns_steer_camber && !has_pou_lut) {
            double c1 = params_.suspension.pou_camber_c1[wheel_index];
            double c2 = params_.suspension.pou_camber_c2[wheel_index];
            camber += c1 * steer_angle + c2 * steer_angle * steer_angle;
        }

        return camber;
    }

    double compute_camber_angle(int wheel_index, double susp_travel_from_eq, double steer_angle) const {
        const std::array<double, 6> zero_loads = {0.0, 0.0, static_Fz_[wheel_index], 0.0, 0.0, 0.0};
        return compute_camber_angle(wheel_index, susp_travel_from_eq, steer_angle, 0.0, zero_loads);
    }

    /// エンジンRPMを取得（デバッグ用）
    double get_engine_rpm(int gear) const {
        double omega_rear_avg = (state_.omega[RL] + state_.omega[RR]) / 2.0;
        return gear_ratios_.wheel_omega_to_engine_rpm(omega_rear_avg, gear);
    }
    
    /// エンジントルクを取得（デバッグ用）
    double get_engine_torque(int gear, double throttle) const {
        double rpm = get_engine_rpm(gear);
        return engine_map_.get_torque(rpm, throttle);
    }

    // ======== Phase 5機能（公開API） ========

    /// アッカーマン幾何による内輪・外輪舵角差計算（§10.2）
    /// @param delta_center 中心舵角 [rad]
    /// @param ackermann_ratio アッカーマン率 (0=パラレル, 1=完全アッカーマン)
    /// @return [delta_FL, delta_FR, 0, 0]
    std::array<double, 4> compute_ackermann_steering(double delta_center, double ackermann_ratio) const {
        std::array<double, 4> steer_angles = {0, 0, 0, 0};
        steer_angles[RL] = 0.0;
        steer_angles[RR] = 0.0;

        if (std::abs(delta_center) < 1e-6) {
            steer_angles[FL] = delta_center;
            steer_angles[FR] = delta_center;
            return steer_angles;
        }

        const double L = params_.mass.a + params_.mass.b;
        const double Tf = params_.mass.track_f;
        const double sign = std::copysign(1.0, delta_center);
        const bool turning_right = (delta_center > 0.0);  // SAE J670e: +Y right, positive steer is right.

        const double R_center = std::abs(L / std::tan(delta_center));
        const double R_inner = std::max(R_center - Tf / 2.0, 1e-6);
        const double R_outer = R_center + Tf / 2.0;

        const double delta_inner_full = sign * std::atan(L / R_inner);
        const double delta_outer_full = sign * std::atan(L / R_outer);

        const double delta_inner = delta_center + ackermann_ratio * (delta_inner_full - delta_center);
        const double delta_outer = delta_center + ackermann_ratio * (delta_outer_full - delta_center);

        if (turning_right) {
            steer_angles[FL] = delta_outer;
            steer_angles[FR] = delta_inner;
        } else {
            steer_angles[FL] = delta_inner;
            steer_angles[FR] = delta_outer;
        }

        return steer_angles;
    }

    /// コンプライアンスステアによる舵角変化計算（§10.3）
    /// @param Mz_front_left 左前輪のセルフアライニングトルク [Nm]
    /// @param Mz_front_right 右前輪のセルフアライニングトルク [Nm]
    /// @return [delta_compliance_FL, delta_compliance_FR, 0, 0]
    std::array<double, 4> compute_compliance_steer(double Mz_front_left, double Mz_front_right) const {
        std::array<double, 4> compliance_angles = {0, 0, 0, 0};
        const double K_steer = params_.steering.stiffness;
        compliance_angles[FL] = Mz_front_left / K_steer;
        compliance_angles[FR] = Mz_front_right / K_steer;
        compliance_angles[RL] = 0.0;
        compliance_angles[RR] = 0.0;
        return compliance_angles;
    }

    /// ペダル入力 [0-1] → ライン圧 [Pa]（テーブル未設定なら線形）
    double compute_brake_pressure_cmd(double pedal) const {
        return interp1_clamped(params_.brake.pedal_table, params_.brake.pressure_table, pedal,
                               pedal * params_.brake.max_pressure);
    }

    /// ライン圧 → 各輪最大ブレーキトルク [Nm]（balance = 前後MC圧比, #36）
    std::array<double, 4> compute_brake_torque_max(double line_pressure) const {
        const auto& b = params_.brake;
        std::array<double, 4> T{};
        for (int i = 0; i < 4; ++i) {
            const double p_axle = 2.0 * line_pressure * ((i < 2) ? b.balance : 1.0 - b.balance);
            T[i] = p_axle * b.caliper_area[i] * b.mu_pad * b.effective_radius[i];
        }
        return T;
    }

    /// ARB力計算
    /// @param susp_travel サスペンション変位 [m]
    /// @return ARB力 [N]
    std::array<double, 4> compute_arb_forces(const std::array<double, 4>& susp_travel) const {
        std::array<double, 4> F_arb = {0, 0, 0, 0};

        double track_f = params_.mass.track_f;
        double roll_diff_f = susp_travel[FL] - susp_travel[FR];
        double roll_angle_f = roll_diff_f / track_f;
        // k_arb is the axle-equivalent roll stiffness [Nm/rad]; the factor 1/2
        // converts it to the equal-and-opposite wheel-force couple.
        // k_arb is at the bar; the wheel sees it through arb_motion_ratio^2 (#10, default 1).
        const double mr_arb_f = params_.suspension.kc_front.arb_motion_ratio;
        const double mr_arb_r = params_.suspension.kc_rear.arb_motion_ratio;
        double torque_arb_f = 0.5 * params_.suspension.k_arb_f * mr_arb_f * mr_arb_f * roll_angle_f;
        double half_track_f = track_f / 2;
        F_arb[FL] = torque_arb_f / half_track_f;
        F_arb[FR] = -torque_arb_f / half_track_f;

        double track_r = params_.mass.track_r;
        double roll_diff_r = susp_travel[RL] - susp_travel[RR];
        double roll_angle_r = roll_diff_r / track_r;
        double torque_arb_r = 0.5 * params_.suspension.k_arb_r * mr_arb_r * mr_arb_r * roll_angle_r;
        double half_track_r = track_r / 2;
        F_arb[RL] = torque_arb_r / half_track_r;
        F_arb[RR] = -torque_arb_r / half_track_r;

        return F_arb;
    }

private:
    Params params_;
    State14DOF state_;
    double time_ = 0;
    double z_equilibrium_ = 0;
    std::array<double, 4> z_u_equilibrium_ = {0, 0, 0, 0};
    std::array<double, 4> static_Fz_ = {0, 0, 0, 0};
    std::array<Eigen::Vector3d, 4> wheel_positions_{};
    
    // サブシステムパラメータ
    physics::tire::TMeasyParams<double> tire_params_;
    std::array<physics::suspension::BoucWenParams<double>, 4> bw_params_;
    std::array<physics::suspension::LuGreParams<double>, 4> lugre_params_;
    
    // パワートレイン
    physics::powertrain::EngineTorqueMap engine_map_;
    physics::powertrain::GearRatios gear_ratios_;
    physics::powertrain::SalisburyLSD lsd_;
    
    // NNタイヤモデル
    mutable physics::tire::NNTextTireModel nn_tire_model_;
    mutable physics::tire::TireThermalIntegrator tire_thermal_;
    mutable bool nn_model_loaded_ = false;
    // Simulation-time ZOH cache. Values use the vehicle/TMeasy
    // tire-coordinate sign convention (after TTC Fy/Mz conversion).
    mutable std::array<physics::tire::TireForces, 4> pinn_force_cache_{};
    mutable std::array<double, 4> pinn_force_cache_time_ = {0.0, 0.0, 0.0, 0.0};
    mutable std::array<bool, 4> pinn_force_cache_valid_ = {false, false, false, false};
    StateVector pinn_cache_state_snapshot_ = StateVector::Zero();
    bool pinn_cache_state_snapshot_valid_ = false;

    // Magic Formula タイヤモデル
    mutable physics::tire::MagicFormulaParams mf_params_;
    mutable bool mf_model_loaded_ = false;
    
    // 4Dエアロマップ
    physics::aero::AeroMap4D<double> aero_map_;

    // DRSシステム
    DRSSystem drs_system_;
    mutable bool drs_active_ = false;
    std::array<double, 4> z_road_ = {0, 0, 0, 0};  // 路面高さ（Z 下向き正）[m]
    double brake_pressure_lagged_ = 0.0;  // ライン圧1次遅れ（step毎ZOH更新）[Pa]
    int last_gear_ = -1;                  // 変速検出用（-1 = 未入力）
    double shift_end_time_ = -1.0;        // 駆動トルク遮断の終了時刻 [s]

    // デバッグ用キャッシュ
    mutable std::array<double, 4> tire_Fz_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Fx_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Fy_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Fx_body_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Fy_body_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Fz_body_ = {0, 0, 0, 0};
    mutable std::array<double, 4> tire_Mz_ = {0, 0, 0, 0};  // セルフアライニングトルク
    mutable std::array<double, 2> kingpin_torque_ = {0, 0};  // 前輪キングピン軸トルク [Nm]
    mutable std::array<double, 4> bw_forces_ = {0, 0, 0, 0};
    mutable std::array<double, 4> lugre_forces_ = {0, 0, 0, 0};
    mutable std::array<double, 4> spring_forces_ = {0, 0, 0, 0};
    mutable std::array<double, 4> pou_displacement_ = {0, 0, 0, 0};  // POU変位 [m]
    mutable std::array<double, 4> pou_camber_ = {0, 0, 0, 0};         // POU誘発キャンバー [rad]

    void invalidate_pinn_force_cache() {
        pinn_force_cache_valid_.fill(false);
        pinn_force_cache_time_.fill(0.0);
        pinn_cache_state_snapshot_valid_ = false;
    }

    void capture_pinn_cache_state_snapshot() {
        pinn_cache_state_snapshot_ = state_to_vector(state_);
        pinn_cache_state_snapshot_valid_ = true;
    }

    /// TTC -> vehicle convention is a Y mirror: Fy, Mx and Mz all flip (#12 adds Mx).
    physics::tire::TireForces to_vehicle_convention(physics::tire::TireForces forces) const {
        if (params_.tire.pinn_ttc_lateral_sign) {
            forces.Fy = -forces.Fy;
            forces.Mx = -forces.Mx;
            forces.Mz = -forces.Mz;
        }
        return forces;
    }

    /// 有効転がり半径 Re = R0 - Fz/(3*k_tire)（#12）。ラジアルタイヤでは Re が無負荷半径から
    /// 静的たわみの約 1/3 だけ減る（Pacejka, Tire and Vehicle Dynamics §1.x の近似）。
    /// ponytail: analytic fit, replace with a TTC RL/RE LUT (tire_fitting/data_loader.py).
    double compute_effective_rolling_radius(double Fz) const {
        return params_.tire.radius - std::max(Fz, 0.0) / (3.0 * params_.tire.k_tire);
    }

    physics::tire::TireForces evaluate_pinn_tire_force(
        int wheel_index,
        const physics::tire::TireInput& input) const {
        const double eval_rate_hz = params_.tire.pinn_eval_rate_hz;
        const bool override_fx_camber =
            params_.tire.pinn_use_camber_input && std::abs(input.IA) > 1e-9;
        auto evaluate = [&]() {
            physics::tire::TireForces forces =
                to_vehicle_convention(nn_tire_model_.predict(input, true));
            if (override_fx_camber) {
                auto no_camber_input = input;
                no_camber_input.IA = 0.0;
                forces.Fx = to_vehicle_convention(
                    nn_tire_model_.predict(no_camber_input, true)).Fx;
            }
            return forces;
        };

        if (eval_rate_hz <= 0.0) {
            return evaluate();
        }

        const double interval = 1.0 / eval_rate_hz;
        const double time_tolerance = 1e-12 * std::max(1.0, std::abs(time_));
        const double sample_time =
            std::floor((time_ + time_tolerance) / interval) * interval;
        const bool sample_due =
            !pinn_force_cache_valid_[wheel_index] ||
            sample_time > pinn_force_cache_time_[wheel_index] + time_tolerance;
        if (sample_due) {
            pinn_force_cache_[wheel_index] = evaluate();
            pinn_force_cache_time_[wheel_index] = sample_time;
            pinn_force_cache_valid_[wheel_index] = true;
        }
        return pinn_force_cache_[wheel_index];
    }

    double compute_driver_tire_steer_angle(const State14DOF& state, const DriverInput& input) const {
        if (params_.steering.torque_mode) {
            return steering_wheel_to_tire_angle(state.delta_sw, params_.steering.R_steer);
        }
        return input.steering;
    }

    static std::string to_lower(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    static std::string resolve_runtime_asset_path(const std::string& path) {
        namespace fs = std::filesystem;
        if (path.empty()) {
            return path;
        }
        const fs::path original(path);
        if (original.is_absolute() || fs::exists(original)) {
            return original.string();
        }
        for (const fs::path& prefix : {fs::path(".."), fs::path("../..")}) {
            const fs::path candidate = prefix / original;
            if (fs::exists(candidate)) {
                return candidate.string();
            }
        }
        return original.string();
    }

    bool wheel_uses_load_sensitive_kinematics(int wheel_index) const {
        const auto& kc = params_.suspension.kc_corners[wheel_index];
        if (!kc.valid) {
            return false;
        }

        for (double value : kc.compliance.camber_per_load) {
            if (std::abs(value) > 1e-14) {
                return true;
            }
        }
        for (double value : kc.compliance.toe_per_load) {
            if (std::abs(value) > 1e-14) {
                return true;
            }
        }
        return false;
    }

    void initialize_selected_tire_model() {
        const std::string tire_model_type = to_lower(params_.tire.model_type);
        nn_model_loaded_ = false;
        mf_model_loaded_ = false;

#ifdef FSAE_TEST_TIRE_MODEL_TYPE
        if (tire_model_type == "tmeasy") {
            return;
        }
#endif
        if (tire_model_type != "pinn" && tire_model_type != "nn") {
            throw std::runtime_error(
                "Production 14DOF tire runtime is PINN-only; tire.model_type must be 'pinn'. "
                "Received: " + params_.tire.model_type);
        }

        if (params_.tire.model_manifest.empty() && params_.tire.model_dir.empty()) {
            throw std::runtime_error(
                "PINN tire runtime requires tire.model_manifest or tire.model_dir; "
                "implicit TMeasy/MagicFormula/default export fallback is disabled.");
        }

        try {
            if (!params_.tire.model_manifest.empty()) {
                nn_tire_model_.load_from_manifest(
                    resolve_runtime_asset_path(params_.tire.model_manifest));
            } else {
                nn_tire_model_.load_from_export(
                    resolve_runtime_asset_path(params_.tire.model_dir));
            }
            nn_model_loaded_ = true;
            std::cout << "Loaded PINN tire model";
            if (!params_.tire.model_name.empty()) {
                std::cout << ": " << params_.tire.model_name;
            }
            std::cout << std::endl;
        } catch (const std::exception& e) {
            std::ostringstream oss;
            oss << "Failed to load selected PINN tire model";
            if (!params_.tire.model_name.empty()) {
                oss << " '" << params_.tire.model_name << "'";
            }
            oss << ": " << e.what();
            throw std::runtime_error(oss.str());
        }
    }
    
    // ======== セットアップ ========
    void setup_tire_params() {
        tire_params_.mu_x_max = params_.tire.mu_x_max;
        tire_params_.mu_y_max = params_.tire.mu_y_max;
        tire_params_.Fz_nom = params_.tire.Fz_nom;
        tire_params_.Lx = params_.tire.relaxation.Lx_base;
        tire_params_.Ly = params_.tire.relaxation.Ly_base;
        tire_params_.v_N = params_.tire.v_N;
        tire_params_.Cx = params_.tire.Cx;
        tire_params_.Cy = params_.tire.Cy;
        tire_params_.dmu_dFz = params_.tire.dmu_dFz;
        tire_params_.pneumatic_trail = params_.tire.pneumatic_trail;
    }
    
    void setup_bouc_wen_params() {
        for (int i = 0; i < 4; ++i) {
            bw_params_[i].c0 = params_.suspension.c0;
            bw_params_[i].k0 = 0.0;
            bw_params_[i].alpha = params_.suspension.bw_alpha;
            bw_params_[i].A = params_.suspension.bw_A;
            bw_params_[i].beta = params_.suspension.bw_beta;
            bw_params_[i].gamma = params_.suspension.bw_gamma;
            bw_params_[i].n = params_.suspension.bw_n;
        }
    }
    
    void setup_lugre_params() {
        for (int i = 0; i < 4; ++i) {
            lugre_params_[i].sigma_0 = params_.suspension.lugre_sigma0;
            lugre_params_[i].sigma_1 = params_.suspension.lugre_sigma1;
            lugre_params_[i].sigma_2 = params_.suspension.lugre_sigma2;
            lugre_params_[i].Fc = params_.suspension.lugre_Fc;
            lugre_params_[i].Fs = params_.suspension.lugre_Fs;
            lugre_params_[i].vs = params_.suspension.lugre_vs;
        }
    }
    
    void setup_powertrain() {
        // エンジントルクカーブの設定
        if (!params_.powertrain.engine_rpm_points.empty() && 
            !params_.powertrain.engine_torque_curve.empty()) {
            engine_map_.set_curve(params_.powertrain.engine_rpm_points, 
                                 params_.powertrain.engine_torque_curve);
        } else {
            // デフォルト: CBR600RR
            engine_map_ = physics::powertrain::create_cbr600rr_engine();
        }
        
        // ギア比の設定
        if (!params_.powertrain.gear_ratios.empty()) {
            gear_ratios_.ratios = params_.powertrain.gear_ratios;
            gear_ratios_.primary_ratio = params_.powertrain.primary_ratio;
            gear_ratios_.final_ratio = params_.powertrain.final_ratio;
        } else {
            // デフォルト: CBR600RR
            gear_ratios_ = physics::powertrain::GearRatios::cbr600rr_default();
        }
        
        // LSDの設定
        physics::powertrain::SalisburyLSD::Params lsd_params;
        lsd_params.ramp_angle_accel = params_.powertrain.lsd_ramp_accel;
        lsd_params.ramp_angle_decel = params_.powertrain.lsd_ramp_decel;
        lsd_params.preload_torque = params_.powertrain.lsd_preload;
        lsd_params.effective_radius = params_.powertrain.lsd_effective_radius;
        lsd_params.clutch_friction = params_.powertrain.lsd_clutch_friction;
        lsd_params.num_plates = params_.powertrain.lsd_num_plates;
        lsd_.set_params(lsd_params);
    }

    void setup_aero_map() {
        aero_map_.params().rho = params_.aero.rho;
        aero_map_.set_reference_conditions(params_.aero.reference_speed, params_.aero.reference_density);

        const double cl_default = -std::abs(params_.aero.CL_A);
        const double cd_default = std::max(params_.aero.CD_A, 0.01);
        aero_map_.set_coefficient_defaults(cl_default, cd_default, params_.aero.aero_balance);

        const std::string mode = to_lower(params_.aero.map_mode);
        if (mode == "direct_load" || mode == "force_map" || mode == "load_map") {
            aero_map_.set_direct_load_defaults(
                params_.aero.Fx_ref_total,
                params_.aero.Fy_ref_total,
                params_.aero.Fz_ref_total,
                params_.aero.Mx_ref_total,
                params_.aero.My_ref_total,
                params_.aero.Mz_ref_total
            );
        }

        if (!params_.aero.aero_map_file.empty()) {
            if (!aero_map_.load_from_json(params_.aero.aero_map_file)) {
                std::cerr << "Failed to load aero map: " << params_.aero.aero_map_file
                          << " (falling back to defaults)" << std::endl;
            }
        }
    }
    
    /// Per-corner static equilibrium including aero load at speed vx (#15, init only).
    /// Linear springs/tyres at the reference point; solves body heave+pitch and unsprung heights
    /// so that spring and tyre forces carry static + aero loads (no start-up bounce).
    void apply_aero_static_equilibrium(double vx) {
        const auto aero = aero_map_.compute_forces(
            std::abs(vx), params_.aero.nominal_front_ride_height_mm,
            params_.aero.nominal_rear_ride_height_mm, 0.0, 0.0, false);
        const double a = params_.mass.a, b = params_.mass.b, L = a + b;
        const double D = -aero.Fz;  // downforce [N], +down
        const double dF_axle[2] = {(D * b - aero.My) / (2.0 * L), (D * a + aero.My) / (2.0 * L)};
        double A[2];
        for (int ax = 0; ax < 2; ++ax) {
            double k_eff = 0.0;
            for (int i = 2 * ax; i < 2 * ax + 2; ++i) {
                const double mr = motion_ratio_at(i, 0.0);
                k_eff += 0.5 * params_.suspension.k_spring[i] * mr * mr;
            }
            A[ax] = dF_axle[ax] / k_eff + dF_axle[ax] / params_.tire.k_tire;  // spring + tyre stroke
        }
        const double theta = (A[1] - A[0]) / L;
        state_.theta = theta;
        state_.z = z_equilibrium_ + A[0] + a * theta;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i] + dF_axle[i / 2] / params_.tire.k_tire;
        }
    }

    void compute_equilibrium() {
        const double m = params_.mass.m_sprung;
        const double g = 9.81;
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double L = a + b;  // wheelbase = a + b
        
        double W = m * g;
        double Wf = W * b / L;
        double Wr = W * a / L;
        
        static_Fz_[FL] = Wf / 2;
        static_Fz_[FR] = Wf / 2;
        static_Fz_[RL] = Wr / 2;
        static_Fz_[RR] = Wr / 2;
        
        // ばね上平衡位置
        double k_avg = 0;
        for (int i = 0; i < 4; ++i) {
            double mr = params_.suspension.motion_ratio[i];
            k_avg += params_.suspension.k_spring[i] * mr * mr;
        }
        k_avg /= 4;
        z_equilibrium_ = W / (4 * k_avg);
        
        // ばね下平衡位置
        double k_tire = params_.tire.k_tire;
        for (int i = 0; i < 4; ++i) {
            double total_load = static_Fz_[i] + params_.mass.m_unsprung[i] * g;
            z_u_equilibrium_[i] = total_load / k_tire;
        }
        
        // 初期状態設定
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
        }
    }
    
    // ======== 状態変換 ========
    StateVector state_to_vector(const State14DOF& s) const {
        StateVector x;
        x << s.x, s.y, s.z, s.phi, s.theta, s.psi,
             s.vx, s.vy, s.vz, s.p, s.q, s.r,
             s.z_u[0], s.z_u[1], s.z_u[2], s.z_u[3],
             s.vz_u[0], s.vz_u[1], s.vz_u[2], s.vz_u[3],
             s.omega[0], s.omega[1], s.omega[2], s.omega[3],
             s.tire_xi[0], s.tire_xi[1], s.tire_xi[2], s.tire_xi[3],
             s.tire_eta[0], s.tire_eta[1], s.tire_eta[2], s.tire_eta[3],
             s.damper_z[0], s.damper_z[1], s.damper_z[2], s.damper_z[3],
             s.friction_z[0], s.friction_z[1], s.friction_z[2], s.friction_z[3],
             s.phi_twist, s.dphi_twist,     // Phase A1
             s.delta_sw, s.ddelta_sw;       // Phase A2
        return x;
    }

    State14DOF vector_to_state(const StateVector& x) const {
        State14DOF s;
        s.x = x(0); s.y = x(1); s.z = x(2);
        s.phi = x(3); s.theta = x(4); s.psi = x(5);
        s.vx = x(6); s.vy = x(7); s.vz = x(8);
        s.p = x(9); s.q = x(10); s.r = x(11);
        for (int i = 0; i < 4; ++i) {
            s.z_u[i] = x(12 + i);
            s.vz_u[i] = x(16 + i);
            s.omega[i] = x(20 + i);
            s.tire_xi[i] = x(24 + i);
            s.tire_eta[i] = x(28 + i);
            s.damper_z[i] = x(32 + i);
            s.friction_z[i] = x(36 + i);
        }
        s.phi_twist = x(40);      // Phase A1
        s.dphi_twist = x(41);     // Phase A1
        s.delta_sw = x(42);       // Phase A2
        s.ddelta_sw = x(43);      // Phase A2
        return s;
    }
    
    // ======== 車輪位置 ========
    std::array<Eigen::Vector3d, 4> build_wheel_positions() const {
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double tf = params_.mass.track_f / 2;
        const double tr = params_.mass.track_r / 2;
        
        return {
            Eigen::Vector3d(a, -tf, 0),   // FL
            Eigen::Vector3d(a, tf, 0),    // FR
            Eigen::Vector3d(-b, -tr, 0),  // RL
            Eigen::Vector3d(-b, tr, 0)    // RR
        };
    }

    const std::array<Eigen::Vector3d, 4>& get_wheel_positions() const {
        return wheel_positions_;
    }
    
    // ======== 回転行列 ========
    std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_rotation_matrices(
        double phi, double theta, double psi) const {
        
        double cphi = std::cos(phi), sphi = std::sin(phi);

        // Euler-rate transformation is singular at theta = +/-90 deg. Apply the
        // same clamp to R and E so the orientation and rate mappings agree.
        const double THETA_SAFE = 1.5184; // 87 deg [rad]
        const double theta_e = std::clamp(theta, -THETA_SAFE, THETA_SAFE);
        double cth = std::cos(theta_e), sth = std::sin(theta_e);
        double cpsi = std::cos(psi), spsi = std::sin(psi);

        Eigen::Matrix3d R;
        R << cpsi*cth, spsi*cth, -sth,
             cpsi*sth*sphi - spsi*cphi, spsi*sth*sphi + cpsi*cphi, cth*sphi,
             cpsi*sth*cphi + spsi*sphi, spsi*sth*cphi - cpsi*sphi, cth*cphi;
        
        // Euler角特異点保護: theta → ±π/2 で tan/cth → ∞
        // 姿勢Rと同じクランプ角を使い、変換の不整合を避ける。
        double tth = std::tan(theta_e);
        Eigen::Matrix3d E;
        E << 1, sphi*tth, cphi*tth,
             0, cphi, -sphi,
             0, sphi/cth, cphi/cth;
        
        return {R, E};
    }
    
    // ======== バンプストップ力 ========
public:  // suspension element helpers (public for tests)
    /// Bump stop on the element stroke x (+ compression), separate bump/rebound gap and rate (#10).
    double compute_bumpstop_force(double x, int /*wheel_idx*/) const {
        const auto& sp = params_.suspension;
        const double gap_rb = sp.bump_stop_gap_rebound < 0.0 ? sp.bump_stop_gap : sp.bump_stop_gap_rebound;
        const double k_rb = sp.bump_stop_k_rebound < 0.0 ? sp.bump_stop_k : sp.bump_stop_k_rebound;
        if (x > sp.bump_stop_gap) return sp.bump_stop_k * (x - sp.bump_stop_gap);
        if (x < -gap_rb) return k_rb * (x + gap_rb);
        return 0;
    }

    /// Motion ratio at wheel travel x from equilibrium (table or constant).
    double motion_ratio_at(int i, double x) const {
        return interp1_clamped(params_.suspension.motion_ratio_table_travel,
                               params_.suspension.motion_ratio_table[i], x,
                               params_.suspension.motion_ratio[i]);
    }

    /// MR applied to damper/LuGre/bump elements (1 = legacy wheel-coordinate elements).
    double damper_motion_ratio(int i, double x) const {
        return params_.suspension.mr_applies_to_dampers ? motion_ratio_at(i, x) : 1.0;
    }

    /// Spring stroke s(x) = ∫_0^x MR(u) du (exact for the piecewise-linear table).
    double spring_stroke(int i, double x) const {
        const auto& t = params_.suspension.motion_ratio_table_travel;
        const auto& m = params_.suspension.motion_ratio_table[i];
        if (t.empty()) return params_.suspension.motion_ratio[i] * x;
        auto integral_from_t0 = [&](double q) {
            if (q <= t[0]) return m[0] * (q - t[0]);
            double acc = 0.0;
            for (size_t k = 1; k < t.size(); ++k) {
                if (q <= t[k]) return acc + 0.5 * (m[k - 1] + motion_ratio_at(i, q)) * (q - t[k - 1]);
                acc += 0.5 * (m[k - 1] + m[k]) * (t[k] - t[k - 1]);
            }
            return acc + m.back() * (q - t.back());
        };
        return integral_from_t0(x) - integral_from_t0(0.0);
    }
private:

    /// @brief フレームねじりを考慮した前後ロール角の計算（Phase A1）
    /// @param phi_body 車体ロール角 [rad]
    /// @param phi_twist フレームねじり角 [rad]
    /// @return {phi_front, phi_rear} 前後の実効ロール角 [rad]
    ///
    /// フレームねじり角 phi_twist は、前後サスペンション間の相対的なロール角変化を表す。
    /// - phi_front = phi_body + phi_twist/2 : 前輪が「見る」ロール角
    /// - phi_rear  = phi_body - phi_twist/2 : 後輪が「見る」ロール角
    std::pair<double, double> compute_split_roll_angles(
        double phi_body, double phi_twist) const {
        double phi_front = phi_body + phi_twist / 2.0;
        double phi_rear = phi_body - phi_twist / 2.0;
        return {phi_front, phi_rear};
    }

    /// @brief アッカーマン幾何とコンプライアンスステアの統合
    /// @param delta_driver ドライバー入力舵角 [rad]
    /// @param Mz_wheels 各輪のセルフアライニングトルク [Nm]
    /// @return 最終的な各輪舵角 [rad]
    std::array<double, 4> compute_total_steering(
        double delta_driver,
        const std::array<double, 4>& Mz_wheels) const
    {
        // 1. アッカーマン幾何による舵角
        auto steer_ackermann = compute_ackermann_steering(delta_driver, params_.steering.ackermann);

        // 2. コンプライアンスステアによる補正（K&C が Mz→toe を持つ輪は二重計上を避けて無効, #6）
        auto steer_compliance = compute_compliance_steer(Mz_wheels[FL], Mz_wheels[FR]);
        for (int i = 0; i < 4; ++i) {
            if (!uses_steering_stiffness_compliance(i)) steer_compliance[i] = 0.0;
        }

        // 3. 合計（ドライバー入力 + コンプライアンス）
        std::array<double, 4> steer_total;
        for (int i = 0; i < 4; ++i) {
            steer_total[i] = steer_ackermann[i] + steer_compliance[i];
        }

        return steer_total;
    }

    double wheel_spin_inertia(int i) const {
        return (params_.mass.Iw_corner[i] > 0.0) ? params_.mass.Iw_corner[i] : params_.mass.Iw;
    }

    /// K_steer path is deprecated in favour of the K&C 6xn compliance matrix; it is
    /// only used where K&C does not already map Mz to toe (#6).
    bool uses_steering_stiffness_compliance(int i) const {
        const auto& kc = params_.suspension.kc_corners[i];
        return i < 2 && !(kc.valid && kc.compliance.toe_per_load[5] != 0.0);
    }

    // ======== キャンバー角・トー角計算（ガイド §2.4） ========
    // compute_camber_angle()とcompute_pou_displacement()はテスト用にpublicに移動済み

public:
    /// @brief 各輪のトー角を計算（テスト用にpublic）
    /// @param wheel_index 車輪インデックス
    /// @return トー角 [rad] (正: トーイン)
    double compute_toe_angle(
        int wheel_index,
        double susp_travel_from_eq,
        double steer_angle,
        double axle_roll_angle,
        const std::array<double, 6>& kc_loads) const {
        double toe = params_.suspension.static_toe[wheel_index];
        const auto& kc = params_.suspension.kc_corners[wheel_index];
        const auto& sv = kc.samples;
        const bool has_travel_lut = kc.valid && sv.travel_m.size() >= 2
            && sv.toe_rad.size() == sv.travel_m.size();
        const bool has_steer_lut = kc.valid && sv.steer_rad.size() >= 2
            && sv.steer_toe_rad.size() == sv.steer_rad.size();
        const bool has_roll_lut = kc.valid && sv.roll_rad.size() >= 2
            && sv.roll_toe_residual_rad.size() == sv.roll_rad.size();
        if (kc.valid) {
            if (has_travel_lut) {
                toe = lut_eval(sv.travel_m, sv.toe_rad, susp_travel_from_eq, toe);
            } else if (!is_zero_polynomial(kc.kinematics.toe_vs_travel)) {
                toe = eval_quadratic(kc.kinematics.toe_vs_travel, susp_travel_from_eq);
            }
            if (has_steer_lut) {
                toe += lut_eval(sv.steer_rad, sv.steer_toe_rad, steer_angle, 0.0);
            } else {
                toe += eval_quadratic(kc.kinematics.toe_vs_steer, steer_angle);
            }
            // Same residual rule as roll_camber_gain (#38).
            if (has_roll_lut) {
                toe += lut_eval(sv.roll_rad, sv.roll_toe_residual_rad, axle_roll_angle, 0.0)
                     - lut_eval(sv.roll_rad, sv.roll_toe_residual_rad, 0.0, 0.0);
            } else if (!has_travel_lut && is_zero_polynomial(kc.kinematics.toe_vs_travel)) {
                toe += kc.kinematics.roll_steer_gain * axle_roll_angle;
            }

            for (int i = 0; i < 6; ++i) {
                toe += kc.compliance.toe_per_load[i] * kc_loads[i];
            }
        }
        return toe;
    }

    double compute_toe_angle(int wheel_index) const {
        const std::array<double, 6> zero_loads = {0.0, 0.0, static_Fz_[wheel_index], 0.0, 0.0, 0.0};
        return compute_toe_angle(wheel_index, 0.0, 0.0, 0.0, zero_loads);
    }
private:

    /// Clamped linear interpolation; returns fallback when the table is absent.
    double lut_eval(const std::vector<double>& x, const std::vector<double>& y,
                    double q, double fallback) const {
        if (x.size() < 2 || x.size() != y.size()) return fallback;
        return interp1_clamped(x, y, q, fallback);
    }

    double compute_effective_relaxation_length(bool longitudinal, double fz, double tire_temp_c) const {
        const auto& relax = params_.tire.relaxation;
        const double base = longitudinal ? relax.Lx_base : relax.Ly_base;
        const double min_len = longitudinal ? relax.min_Lx : relax.min_Ly;
        const double max_len = longitudinal ? relax.max_Lx : relax.max_Ly;
        if (!relax.enabled) {
            return std::clamp(base, min_len, max_len);
        }

        const double fz_ratio = std::max(fz, 1.0) / std::max(params_.tire.Fz_nom, 1.0);
        const double load_scale = std::pow(fz_ratio, relax.Fz_exponent);
        const double temp_scale = 1.0 + relax.temp_coeff_per_c * (tire_temp_c - relax.temp_ref_c);
        return std::clamp(base * load_scale * temp_scale, min_len, max_len);
    }

    double compute_roll_center_height_for_wheel(int wheel_index, const std::array<double, 4>& susp_travel_from_eq, double body_roll_angle) const {
        const auto& axle = (wheel_index < 2) ? params_.suspension.kc_front : params_.suspension.kc_rear;
        const auto& sv = params_.suspension.kc_corners[wheel_index].samples;
        if (sv.travel_m.size() >= 2 && sv.rc_height_m.size() == sv.travel_m.size()) {
            double h = lut_eval(sv.travel_m, sv.rc_height_m,
                                susp_travel_from_eq[wheel_index], axle.roll_center_height);
            if (sv.roll_rad.size() >= 2 && sv.roll_center_height_m.size() == sv.roll_rad.size()) {
                h += lut_eval(sv.roll_rad, sv.roll_center_height_m, body_roll_angle, 0.0)
                   - lut_eval(sv.roll_rad, sv.roll_center_height_m, 0.0, 0.0);
            } else {
                h += axle.roll_center_height_per_roll * body_roll_angle;
            }
            return h;
        }
        const double axle_travel = (wheel_index < 2)
            ? 0.5 * (susp_travel_from_eq[FL] + susp_travel_from_eq[FR])
            : 0.5 * (susp_travel_from_eq[RL] + susp_travel_from_eq[RR]);
        return axle.roll_center_height
             + axle.roll_center_height_per_travel * axle_travel
             + axle.roll_center_height_per_roll * body_roll_angle;
    }

    /// Vertical link force [N, upward on body] carried by the suspension links (#9).
    ///
    /// The tyre forces act on the body with the full h_cg arm; the links carry
    ///   - lateral: the roll-centre (geometric) share, F = k_jack*Fy with
    ///     k_jack = jacking_force_per_fy (K&C) or derived -sign(y)*h_rc/(t/2),
    ///   - longitudinal: the anti share, F = -/+ Fx*anti*h_cg/L (front/rear).
    /// Added to F_susp like a spring force, so the total load transfer stays
    /// m*a*h_cg/(L or t) independent of RC height / anti% — they only move the
    /// split between springs (elastic) and links (geometric).
    double compute_link_force(int i, double fx_body, double fy_body,
                              const std::array<double, 4>& susp_travel_from_eq,
                              double axle_roll_angle) const {
        const bool front = i < 2;
        const auto& kc = params_.suspension.kc_corners[i];
        const auto& sv = kc.samples;
        const auto& axle = front ? params_.suspension.kc_front : params_.suspension.kc_rear;
        double k_jack = (kc.valid && kc.kinematics.jacking_force_per_fy != 0.0)
            ? kc.kinematics.jacking_force_per_fy
            : 0.0;
        if (sv.travel_m.size() >= 2 && sv.jacking_force_per_fy.size() == sv.travel_m.size()) {
            k_jack = lut_eval(sv.travel_m, sv.jacking_force_per_fy,
                              susp_travel_from_eq[i], k_jack);
        } else if (k_jack == 0.0 && axle.roll_center_height != 0.0) {
            // ponytail: linear RC migration, swap for RCz(heave, roll) LUT from the K&C sweep.
            const double half_track = 0.5 * (front ? params_.mass.track_f : params_.mass.track_r);
            const double h_rc = compute_roll_center_height_for_wheel(i, susp_travel_from_eq, axle_roll_angle);
            k_jack = -std::copysign(1.0, wheel_positions_[i].y()) * h_rc / half_track;
        }
        double anti = fx_body >= 0.0 ? axle.anti_drive_pct : axle.anti_brake_pct;
        const auto& anti_lut = fx_body >= 0.0 ? sv.anti_drive : sv.anti_brake;
        if (sv.travel_m.size() >= 2 && anti_lut.size() == sv.travel_m.size()) {
            anti = lut_eval(sv.travel_m, anti_lut, susp_travel_from_eq[i], anti);
        }
        anti = std::clamp(anti, 0.0, 1.0);
        const double L = params_.mass.a + params_.mass.b;
        const double f_anti = (front ? -1.0 : 1.0) * fx_body * anti * params_.mass.h_cg / L;
        return k_jack * fy_body + f_anti;
    }

    /// @brief タイヤ座標系への完全な座標変換行列を計算
    /// @param steer_angle ステアリング角 [rad]
    /// @param camber_angle キャンバー角 [rad]
    /// @param toe_angle トー角 [rad]
    /// @return 車体座標系からタイヤ座標系への回転行列
    Eigen::Matrix3d compute_wheel_rotation_matrix(
        double steer_angle, double camber_angle, double toe_angle) const
    {
        // 回転順序: Z軸(ステア+トー) → X軸(キャンバー)
        // 合計舵角 = ステアリング + トー
        double total_yaw = steer_angle + toe_angle;

        double cy = std::cos(total_yaw);
        double sy = std::sin(total_yaw);
        double cc = std::cos(camber_angle);
        double sc = std::sin(camber_angle);

        // Z軸周り回転(ヨー) -> X軸周り回転(キャンバー) での 車体系からタイヤ系への変換
        // R_wheel = R_x(camber) * R_z(yaw)
        Eigen::Matrix3d R_wheel;
        R_wheel <<  cy,       sy,       0.0,
                   -sy * cc,  cy * cc,  sc,
                    sy * sc, -cy * sc,  cc;

        return R_wheel;
    }

    /// @brief サスペンションによるロールモーメントを前後別に計算（Phase A1）
    /// @param F_susp 各輪のサスペンション垂直力 [N]（4要素配列）
    /// @param wheel_pos 各輪の車体座標系位置 [m]（4要素配列）
    /// @return {M_front, M_rear} 前後のロールモーメント [Nm]
    ///
    /// フレームねじり剛性により、前後サスペンションは異なるロール角を経験する。
    /// この関数は、前後のサスペンション力によるロールモーメントを分離計算する。
    /// - M_front: 前輪（FL, FR）によるロールモーメント
    /// - M_rear:  後輪（RL, RR）によるロールモーメント
    /// 符号規約: 負値 = 車体を右に傾ける（右旋回時の遠心力に対抗）
    std::pair<double, double> compute_suspension_roll_moments(
        const std::array<double, 4>& F_susp,
        const std::array<Eigen::Vector3d, 4>& wheel_pos) const {

        double M_front = -(wheel_pos[FL].y() * F_susp[FL] + wheel_pos[FR].y() * F_susp[FR]);
        double M_rear = -(wheel_pos[RL].y() * F_susp[RL] + wheel_pos[RR].y() * F_susp[RR]);

        return {M_front, M_rear};
    }

    // ======== メイン微分計算 ========
    StateVector compute_derivatives(const State14DOF& state, const DriverInput& input, bool update_cache = false) const {
        using namespace physics::tire;
        using namespace physics::suspension;
        
        StateVector dxdt = StateVector::Zero();
        
        // 定数
        const double g = 9.81;
        const double m = params_.mass.m_sprung;
        // Unsprung masses translate with the body in x/y, so the planar EOM
        // uses the full vehicle mass (#3). Summed from the modelled masses so
        // static loads and translation stay consistent (== mass.m_total in artifacts).
        const double m_trans = m + params_.mass.m_unsprung[0] + params_.mass.m_unsprung[1]
                                 + params_.mass.m_unsprung[2] + params_.mass.m_unsprung[3];
        const double Ixx = params_.mass.Ixx;
        const double Iyy = params_.mass.Iyy;
        const double Izz = params_.mass.Izz;
        const double k_tire = params_.tire.k_tire;
        const double c_tire = params_.tire.c_tire;
        const double h_cg = params_.mass.h_cg;
        
        // 車輪位置
        const auto& wheel_pos = get_wheel_positions();
        
        // 回転行列
        const auto [R, E] = compute_rotation_matrices(state.phi, state.theta, state.psi);
        
        // ======== 運動学 ========
        Eigen::Vector3d v_body(state.vx, state.vy, state.vz);
        Eigen::Vector3d v_inertial = R.transpose() * v_body;
        dxdt(0) = v_inertial.x();
        dxdt(1) = v_inertial.y();
        dxdt(2) = v_inertial.z();
        
        Eigen::Vector3d omega_body(state.p, state.q, state.r);
        Eigen::Vector3d euler_rate = E * omega_body;
        dxdt(3) = euler_rate(0);
        dxdt(4) = euler_rate(1);
        dxdt(5) = euler_rate(2);
        
        // ======== ステアリング（アッカーマン幾何 + コンプライアンスステア） ========
        // ガイド §10.2, §10.3

        // Phase A2: トルクモード vs 角度モード
        double delta_driver;  // ドライバー要求舵角（タイヤ座標）

        if (params_.steering.torque_mode) {
            // トルクモード: ステアリングホイール角から変換
            delta_driver = steering_wheel_to_tire_angle(state.delta_sw, params_.steering.R_steer);
        } else {
            // 角度モード（後方互換性）: max_angle でクランプ（#7）
            delta_driver = std::clamp(input.steering, -params_.steering.max_angle,
                                      params_.steering.max_angle);
        }

        // 前ステップの Mz を初期値にし、K&C fixed-point ループ内で当該 stage の Mz で更新（#6: 1 step lag 解消）
        const auto steer_ack = compute_ackermann_steering(delta_driver, params_.steering.ackermann);
        std::array<double, 4> steer = compute_total_steering(delta_driver, tire_Mz_);
        
        // ======== フレームねじりによるロール角分離（Phase A1）========
        // 剛体モード: K_chassis >= 1e5 Nm/deg では ω_n >> RK4安定限界のため
        // phi_twist/dphi_twistを完全にゼロとして扱う
        const bool rigid_frame = (params_.suspension.K_chassis >= 1e5);
        const double phi_twist_eff = rigid_frame ? 0.0 : state.phi_twist;
        const double dphi_twist_eff = rigid_frame ? 0.0 : state.dphi_twist;

        auto [phi_front, phi_rear] = compute_split_roll_angles(state.phi, phi_twist_eff);

        // ======== 各輪の力計算 ========
        std::array<double, 4> Fx_wheel, Fy_wheel, F_spring, F_bw, F_lugre, F_bump, F_jacking;
        std::array<double, 4> tire_Fx_stage = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> tire_Fy_stage = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> tire_Mz_stage = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> Fz_tire, susp_travel, dz_bw, dz_lugre;
        std::array<double, 4> susp_travel_from_eq_arr = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> R_eff{};
        std::array<double, 4> tire_Mx_stage = {0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> tire_Fz_body_stage = {0.0, 0.0, 0.0, 0.0};

        for (int i = 0; i < 4; ++i) {
            // 各輪が「見る」ロール角（フレームねじりを考慮）
            double phi_at_wheel = (i < 2) ? phi_front : phi_rear;

            // サスペンション変位（分離されたロール角を使用）
            double z_sprung_at_wheel = state.z
                - wheel_pos[i].x() * std::sin(state.theta)
                + wheel_pos[i].y() * std::sin(phi_at_wheel);
            susp_travel[i] = z_sprung_at_wheel - state.z_u[i];

            // サスペンション速度（フレームねじり速度も考慮）
            double dphi_at_wheel = (i < 2) ? (state.p + dphi_twist_eff / 2.0)
                                            : (state.p - dphi_twist_eff / 2.0);
            double vz_sprung_at_wheel = state.vz
                - wheel_pos[i].x() * state.q * std::cos(state.theta)
                + wheel_pos[i].y() * dphi_at_wheel * std::cos(phi_at_wheel);
            double susp_velocity = vz_sprung_at_wheel - state.vz_u[i];
            
            // 平衡からの偏差
            double susp_travel_from_eq = susp_travel[i] - (z_equilibrium_ - z_u_equilibrium_[i]);
            susp_travel_from_eq_arr[i] = susp_travel_from_eq;

            // POU: ステアリングによる強制変位をスプリングストロークに加算
            double pou_disp = compute_pou_displacement(i, susp_travel_from_eq, steer[i]);
            double effective_susp_travel_from_eq = susp_travel_from_eq + pou_disp;

            // POU変位を保存（デバッグ用）
            if (update_cache) {
                pou_displacement_[i] = pou_disp;
            }

            // スプリング力（pou_dispを反映）: F_wheel = MR(x)*(F_static/MR(0) + k*∫MR) (#10)
            const double x_eff = effective_susp_travel_from_eq;
            const double mr = pou_mr_at(i, x_eff, steer[i]);
            const double mr_static = pou_mr_at(i, 0.0, steer[i]);
            const double mr_static_safe = std::abs(mr_static) > 1e-9
                ? mr_static : motion_ratio_at(i, 0.0);
            F_spring[i] = mr * (static_Fz_[i] / mr_static_safe
                                + params_.suspension.k_spring[i] * spring_stroke(i, x_eff));
            if (update_cache) {
                spring_forces_[i] = F_spring[i];
            }

            // 素子座標で評価し MR 倍でホイールへ: F_wheel = MR*F_elem(MR*x, MR*v) (#10)
            const double mr_d = pou_damper_mr_at(i, x_eff, steer[i]);

            // Bouc-Wenダンパー力（変位にPOU反映、速度はquasi-static近似で変更なし）
            auto bw_out = compute_bouc_wen(bw_params_[i], mr_d * x_eff, mr_d * susp_velocity, state.damper_z[i]);
            const auto& damper_v = params_.suspension.damper_lut_velocity;
            const auto& damper_f = params_.suspension.damper_lut_force[i];
            if (damper_v.size() >= 2 && damper_f.size() == damper_v.size()) {
                // Measured dyno LUT at the element, 4-wheel independent (#11).
                F_bw[i] = mr_d * interp1_clamped(damper_v, damper_f, mr_d * susp_velocity,
                                                 bw_out.force);
                dz_bw[i] = 0.0;
            } else {
                F_bw[i] = mr_d * bw_out.force;
                dz_bw[i] = bw_out.dz_dt;
            }
            if (update_cache) {
                bw_forces_[i] = F_bw[i];
            }

            // LuGre摩擦力
            auto lugre_out = compute_lugre(lugre_params_[i], mr_d * susp_velocity, state.friction_z[i]);
            F_lugre[i] = mr_d * lugre_out.force;
            dz_lugre[i] = lugre_out.dz_dt;
            if (update_cache) {
                lugre_forces_[i] = F_lugre[i];
            }

            // バンプストップ力（POU反映）
            F_bump[i] = mr_d * compute_bumpstop_force(mr_d * x_eff, i);
            
            // タイヤ垂直力
            // ponytail: road height only (no road normal / dz_road/dt in tyre damping); add for kerbs.
            double tire_deflection = state.z_u[i] - z_road_[i] - z_u_equilibrium_[i];
            double tire_velocity = state.vz_u[i];
            Fz_tire[i] = static_Fz_[i] + params_.mass.m_unsprung[i] * g
                       + k_tire * tire_deflection + c_tire * tire_velocity;
            Fz_tire[i] = std::max(Fz_tire[i], 0.0);
            if (update_cache) {
                tire_Fz_[i] = Fz_tire[i];
            }
            R_eff[i] = compute_effective_rolling_radius(Fz_tire[i]);
            
            // タイヤ滑り計算（完全な座標変換）
            // ガイド §2.4: 車体系 → サスペンション系 → ホイール系 → タイヤ接地点系

            // 2. K&C load coupling は各 RK4 stage の trial state に閉じるよう
            //    wheel ごとに簡易 fixed-point を 2 回だけ回す
            const double axle_roll_angle = (i < 2) ? phi_front : phi_rear;
            std::array<double, 6> kc_loads = {0.0, 0.0, Fz_tire[i], 0.0, 0.0, 0.0};
            double camber = params_.suspension.static_camber[i];
            double toe = params_.suspension.static_toe[i];

            const bool ksteer = uses_steering_stiffness_compliance(i);
            const int kc_iterations = (wheel_uses_load_sensitive_kinematics(i) || ksteer) ? 2 : 1;
            for (int kc_iter = 0; kc_iter < kc_iterations; ++kc_iter) {
                if (kc_iter > 0 && ksteer) {
                    steer[i] = steer_ack[i] + tire_Mz_stage[i] / params_.steering.stiffness;
                }
                // Kinematics follow the wheel-to-body stroke (state), not the
                // POU pushrod-length change (#38); springs keep effective_*.
                camber = compute_camber_angle(i, susp_travel_from_eq, steer[i], axle_roll_angle, kc_loads);
                toe = compute_toe_angle(i, susp_travel_from_eq, steer[i], axle_roll_angle, kc_loads);

                // Load compliance moves the contact patch (#7). track_per_load is the
                // per-corner track change; mirror it to the wheel's side.
                double track_delta = 0.0, wheelbase_delta = 0.0;
                for (int k = 0; k < 6; ++k) {
                    track_delta += params_.suspension.kc_corners[i].compliance.track_per_load[k] * kc_loads[k];
                    wheelbase_delta += params_.suspension.kc_corners[i].compliance.wheelbase_per_load[k] * kc_loads[k];
                }
                const auto& corner_samples = params_.suspension.kc_corners[i].samples;
                if (corner_samples.travel_m.size() >= 2
                    && corner_samples.track_change_m.size() == corner_samples.travel_m.size()) {
                    track_delta += lut_eval(corner_samples.travel_m, corner_samples.track_change_m,
                                            susp_travel_from_eq, 0.0);
                }
                const Eigen::Vector3d contact_position(
                    wheel_pos[i].x() + wheelbase_delta,
                    wheel_pos[i].y() + std::copysign(1.0, wheel_pos[i].y()) * track_delta,
                    params_.mass.h_cg);
                const Eigen::Vector3d contact_velocity = compute_contact_patch_velocity(
                    Eigen::Vector3d(state.vx, state.vy, state.vz),
                    Eigen::Vector3d(state.p, state.q, state.r),
                    contact_position);
                const double vx_contact = contact_velocity.x();
                const double vy_contact = contact_velocity.y();

                // POUによるキャンバー変化を保存（デバッグ用: 総キャンバーから静的分を引く）
                double static_camber_component = params_.suspension.static_camber[i]
                                               + params_.suspension.camber_gain[i] * susp_travel_from_eq;
                if (update_cache) {
                    pou_camber_[i] = camber - static_camber_component;
                }

                // 3. 完全な回転行列による座標変換
                // （ステアリング + トー + キャンバー）
                Eigen::Matrix3d R_wheel = compute_wheel_rotation_matrix(steer[i], camber, toe);
                Eigen::Vector3d v_body_vec(vx_contact, vy_contact, 0.0);
                Eigen::Vector3d v_wheel_vec = R_wheel * v_body_vec;

                double vx_wheel = v_wheel_vec.x();
                double vy_wheel = v_wheel_vec.y();
                
                double omega_r = state.omega[i] * R_eff[i];
                // One slip definition for all tire models (#20). TMeasy consumes
                // the same transport speed and slip velocities internally.
                const CommonSlip slip = common_slip(vx_wheel, vy_wheel, omega_r);
                double vsx = -slip.transport * slip.kappa;
                double vsy = vy_wheel;
                
                // タイヤ力計算（NN → MF → TMeasy フォールバック）
                if (nn_model_loaded_) {
                    // === NN タイヤモデル ===

                    const double SR_inst = slip.kappa;
                    const double SA_inst = slip.alpha_rad * 180.0 / M_PI;

                    // Camber-aware PINN path. The left/right IA mirroring
                    // follows the TTC sign convention; Fx is evaluated at IA=0
                    // in evaluate_pinn_tire_force.
                    const bool is_left_wheel = (i == FL || i == RL);
                    double IA = params_.tire.pinn_use_camber_input
                        ? (is_left_wheel ? -camber : camber) * 180.0 / M_PI
                        : 0.0;

                    // タイヤ表面温度（TSTC）[°C]
                    double TSTC = tire_thermal_.get_temperature(i);

                    const double sigma_x = compute_effective_relaxation_length(true, Fz_tire[i], TSTC);
                    const double sigma_y = compute_effective_relaxation_length(false, Fz_tire[i], TSTC);
                    const double transport_velocity = slip.transport;

                    const double kappa_lag = state.tire_xi[i];
                    const double alpha_lag = state.tire_eta[i];
                    dxdt(24 + i) = (transport_velocity / sigma_x) * (SR_inst - kappa_lag);
                    dxdt(28 + i) = (transport_velocity / sigma_y) * (SA_inst - alpha_lag);

                    // NN推論
                    physics::tire::TireInput nn_input;
                    nn_input.SR = kappa_lag;
                    nn_input.SA = alpha_lag;
                    nn_input.Fz = Fz_tire[i];
                    nn_input.IA = IA;
                    nn_input.TSTC = TSTC;

                    const auto nn_forces = evaluate_pinn_tire_force(i, nn_input);

                    // 車体座標系へ変換（接地面反力 Fz も含めた 3D 回転, #21）
                    Eigen::Vector3d F_tire_vec(nn_forces.Fx, nn_forces.Fy, -Fz_tire[i]);
                    Eigen::Vector3d F_body_vec = R_wheel.transpose() * F_tire_vec;
                    Fx_wheel[i] = F_body_vec.x();
                    Fy_wheel[i] = F_body_vec.y();
                    tire_Fz_body_stage[i] = F_body_vec.z();
                    tire_Fx_stage[i] = nn_forces.Fx;
                    tire_Fy_stage[i] = nn_forces.Fy;
                    tire_Mz_stage[i] = nn_forces.Mz;
                    tire_Mx_stage[i] = nn_forces.Mx;

                } else if (mf_model_loaded_) {
                    // === Magic Formula タイヤモデル (with Relaxation Length) ===
                    // ガイド §5.4: 過渡特性（緩和長）の実装
                    
                    // 1. 共通の瞬間スリップ（#20）
                    const double kappa_inst = slip.kappa;
                    const double alpha_inst_rad = slip.alpha_rad;

                    // 2. 状態変数から遅れスリップを取得
                    // tire_xi -> Lagged Kappa (kappa_lag)
                    // tire_eta -> Lagged Alpha (alpha_lag) [rad]
                    double kappa_lag = state.tire_xi[i];
                    double alpha_lag_rad = state.tire_eta[i];
                    
                    // 3. 緩和長による状態微分（First-Order Lag）
                    // dS/dt = (V / L) * (S_inst - S_lag)
                    const double tire_temp = tire_thermal_.get_temperature(i);
                    double Lx = compute_effective_relaxation_length(true, Fz_tire[i], tire_temp);
                    double Ly = compute_effective_relaxation_length(false, Fz_tire[i], tire_temp);
                    const double V_transport = slip.transport;
                    
                    double d_kappa = (V_transport / Lx) * (kappa_inst - kappa_lag);
                    double d_alpha = (V_transport / Ly) * (alpha_inst_rad - alpha_lag_rad);
                    
                    dxdt(24 + i) = d_kappa;
                    dxdt(28 + i) = d_alpha;

                    // 4. 遅れスリップを用いて力を計算
                    double IA = camber * 180.0 / M_PI; // [deg]
                    double gamma_rad = camber;         // [rad]
                    double Fz_mf = -Fz_tire[i];        // TTC convention (Fz < 0)
                    
                    // MF計算には遅れスリップを使用
                    auto mf_forces = compute_magic_formula(mf_params_, kappa_lag, alpha_lag_rad, Fz_mf, gamma_rad);

                    const auto vehicle_forces = magic_formula_to_vehicle_convention(mf_forces);

                    // 車体座標系へ変換（接地面反力 Fz も含めた 3D 回転, #21）
                    Eigen::Vector3d F_tire_vec_mf(vehicle_forces.Fx, vehicle_forces.Fy, -Fz_tire[i]);
                    Eigen::Vector3d F_body_vec_mf = R_wheel.transpose() * F_tire_vec_mf;
                    Fx_wheel[i] = F_body_vec_mf.x();
                    Fy_wheel[i] = F_body_vec_mf.y();
                    tire_Fz_body_stage[i] = F_body_vec_mf.z();
                    tire_Fx_stage[i] = vehicle_forces.Fx;
                    tire_Fy_stage[i] = vehicle_forces.Fy;
                    tire_Mz_stage[i] = vehicle_forces.Mz;

                } else {
                    // === TMeasy フォールバック ===
                    TMeasyState<double> tire_state;
                    tire_state.xi = state.tire_xi[i];
                    tire_state.eta = state.tire_eta[i];

                    auto tire_params_dynamic = tire_params_;
                    const double tire_temp = tire_thermal_.get_temperature(i);
                    tire_params_dynamic.Lx = compute_effective_relaxation_length(true, Fz_tire[i], tire_temp);
                    tire_params_dynamic.Ly = compute_effective_relaxation_length(false, Fz_tire[i], tire_temp);

                    auto tire_out = compute_tmeasy(tire_params_dynamic, tire_state, Fz_tire[i],
                                                  vsx, vsy, slip.transport);

                    // 車体座標系へ変換（接地面反力 Fz も含めた 3D 回転, #21）
                    Eigen::Vector3d F_tire_vec_tm(tire_out.Fx, tire_out.Fy, -Fz_tire[i]);
                    Eigen::Vector3d F_body_vec_tm = R_wheel.transpose() * F_tire_vec_tm;
                    Fx_wheel[i] = F_body_vec_tm.x();
                    Fy_wheel[i] = F_body_vec_tm.y();
                    tire_Fz_body_stage[i] = F_body_vec_tm.z();
                    tire_Fx_stage[i] = tire_out.Fx;
                    tire_Fy_stage[i] = tire_out.Fy;
                    tire_Mz_stage[i] = tire_out.Mz;

                    // タイヤ変形微分
                    dxdt(24 + i) = tire_out.dxi_dt;
                    dxdt(28 + i) = tire_out.deta_dt;
                }

                kc_loads = {tire_Fx_stage[i], tire_Fy_stage[i], Fz_tire[i], tire_Mx_stage[i], 0.0, tire_Mz_stage[i]};
            }

            if (update_cache) {
                tire_Fx_[i] = tire_Fx_stage[i];
                tire_Fy_[i] = tire_Fy_stage[i];
                tire_Fx_body_[i] = Fx_wheel[i];
                tire_Fy_body_[i] = Fy_wheel[i];
                tire_Fz_body_[i] = tire_Fz_body_stage[i];
                tire_Mz_[i] = tire_Mz_stage[i];
            }
            
            
            // Bouc-Wen/LuGre微分
            dxdt(32 + i) = dz_bw[i];
            dxdt(36 + i) = dz_lugre[i];
        }

        for (int i = 0; i < 4; ++i) {
            F_jacking[i] = compute_link_force(i, Fx_wheel[i], Fy_wheel[i], susp_travel_from_eq_arr,
                                              (i < 2) ? phi_front : phi_rear);
        }
        
        // ======== ARB力 ========
        // ばね下にも作用するため、ばね下EOM前に計算
        auto F_arb = compute_arb_forces(susp_travel);
        
        // ======== 合計力・モーメント ========
        double Fx_total = 0, Fy_total = 0, Fz_total = 0;
        std::array<double, 4> F_susp_each{};
        std::array<double, 4> roll_moment_arm{};
        std::array<double, 4> pitch_moment_arm{};

        for (int i = 0; i < 4; ++i) {
            Fx_total += Fx_wheel[i];
            Fy_total += Fy_wheel[i];

            const double F_susp = F_spring[i] + F_bw[i] + F_lugre[i] + F_bump[i] + F_jacking[i] + F_arb[i];
            F_susp_each[i] = F_susp;
            Fz_total -= F_susp;

            // Full arms: RC height and anti% enter as link forces in F_susp (#9).
            roll_moment_arm[i] = h_cg;
            pitch_moment_arm[i] = h_cg;
        }

        const auto body_moments = compute_body_moment_contributions(
            wheel_pos, Fx_wheel, Fy_wheel, F_susp_each,
            roll_moment_arm, pitch_moment_arm, tire_Mz_stage);
        // Tyre overturning moment is reacted through the suspension into the body (#12).
        double Mx_total = body_moments.Mx + tire_Mx_stage[0] + tire_Mx_stage[1]
                        + tire_Mx_stage[2] + tire_Mx_stage[3];
        double My_total = body_moments.My;
        double Mz_total = body_moments.Mz;

        // Wheel gyroscopic moments (#13); front-wheel kingpin (z) part also feeds steering.
        std::array<double, 2> T_gyro_kingpin{};
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector3d M_gyro = compute_wheel_gyro_moment(
                wheel_spin_inertia(i), state.omega[i], (i < 2) ? steer[i] : 0.0, omega_body);
            Mx_total += M_gyro.x();
            My_total += M_gyro.y();
            Mz_total += M_gyro.z();
            if (i < 2) T_gyro_kingpin[i] = M_gyro.z();
        }
        
        // ======== 空力（4D AeroMap） ========
        double v_magnitude = std::sqrt(state.vx * state.vx + state.vy * state.vy);
        
        // ライドハイト計算（ピッチ・ロール姿勢を反映）
        const double static_front_travel =
            0.5 * ((z_equilibrium_ - z_u_equilibrium_[FL]) + (z_equilibrium_ - z_u_equilibrium_[FR]));
        const double static_rear_travel =
            0.5 * ((z_equilibrium_ - z_u_equilibrium_[RL]) + (z_equilibrium_ - z_u_equilibrium_[RR]));
        const double current_front_travel = 0.5 * (susp_travel[FL] + susp_travel[FR]);
        const double current_rear_travel = 0.5 * (susp_travel[RL] + susp_travel[RR]);

        // z は下向き正なので、サスペンション圧縮増加は ride height 低下に対応する
        double rh_front = params_.aero.nominal_front_ride_height_mm
                        - (current_front_travel - static_front_travel) * 1000.0;
        double rh_rear = params_.aero.nominal_rear_ride_height_mm
                       - (current_rear_travel - static_rear_travel) * 1000.0;
        
        // ロール角（rad）
        double roll_rad = state.phi;
        
        // ヨー角（車体スリップ角、rad）
        double beta = (v_magnitude > 0.1) ? std::atan2(state.vy, state.vx) : 0.0;
        
        // 4Dエアロマップから力を計算（DRS状態を考慮）
        // 引数順: (velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, drs_active)
        auto aero_result = aero_map_.compute_forces(v_magnitude, rh_front, rh_rear, roll_rad, beta, drs_active_);
        
        Fx_total += aero_result.Fx;  // ドラッグ（負）
        Fz_total -= aero_result.Fz;  // ダウンフォース（負の力を減算して、正の下向き力にする）
        
        // サイドフォース（ヨー角効果）
        double Fy_aero = aero_result.Fy;
        Fy_total += Fy_aero;
        
        // エアロモーメント
        Mx_total += aero_result.Mx;  // ロールモーメント
        My_total += aero_result.My;  // ピッチモーメント
        Mz_total += aero_result.Mz;  // ヨーモーメント
        
        // ======== ばね下横慣性（#3 第二段階）========
        // The unsprung lateral inertia m_u*a_y is reacted directly at the tyres
        // (arm = wheel-centre height), not through the sprung roll moment.
        const double a_lat = Fy_total / m_trans;
        const double R_wc = params_.tire.radius;
        std::array<double, 4> F_unsprung_lt{};
        for (int i = 0; i < 4; ++i) {
            Mx_total += params_.mass.m_unsprung[i] * a_lat * roll_moment_arm[i];
            const int j = (i < 2) ? 1 - i : 5 - i;  // same-axle partner
            const double track = (i < 2) ? params_.mass.track_f : params_.mass.track_r;
            const double m_axle = params_.mass.m_unsprung[i] + params_.mass.m_unsprung[j];
            // y<0 (left) is the outer wheel for a_lat>0 (right turn): load increases.
            F_unsprung_lt[i] = -std::copysign(1.0, wheel_pos[i].y()) * m_axle * a_lat * R_wc / track;
        }

        // ばね下運動方程式
        for (int i = 0; i < 4; ++i) {
            double m_u = params_.mass.m_unsprung[i];
            double F_susp_on_unsprung = F_spring[i] + F_bw[i] + F_lugre[i] + F_bump[i] + F_jacking[i] + F_arb[i];
            // Rotated contact force: with camber/steer the vertical load also has
            // body x/y components and Fx/Fy feed the body-z force (#21).
            double az_u = g + (F_susp_on_unsprung + F_unsprung_lt[i] + tire_Fz_body_stage[i]) / m_u;

            dxdt(12 + i) = state.vz_u[i];
            dxdt(16 + i) = az_u;
        }

        // ======== 重力 ========
        Eigen::Vector3d g_inertial(0, 0, g);
        Eigen::Vector3d g_body = R * g_inertial;
        
        // ======== ばね上運動方程式 ========
        double ax = Fx_total / m_trans + g_body.x() + state.r * state.vy - state.q * state.vz;
        double ay = Fy_total / m_trans + g_body.y() - state.r * state.vx + state.p * state.vz;
        double az = Fz_total / m + g_body.z() + state.q * state.vx - state.p * state.vy;
        
        dxdt(6) = ax;
        dxdt(7) = ay;
        dxdt(8) = az;

        // ======== 回転運動方程式（完全オイラー方程式） ========
        // ガイド § 4.3 に基づく完全版（ジャイロスコピック項 + 慣性乗積連成）
        //
        // オリジナル方程式:
        //   I_xx·dp - I_xz·(dr + pq) + (I_zz - I_yy)qr = L_ext  ... (1)
        //   I_yy·dq + (I_xx - I_zz)pr + I_xz(p² - r²) = M_ext   ... (2)
        //   I_zz·dr - I_xz·(dp - qr) + (I_yy - I_xx)pq = N_ext  ... (3)
        //
        // (1)と(3)は dp, dr について連立しているため、行列形式で解く:
        //   [I_xx  -I_xz] [dp]   [L_ext + I_xz·pq - (I_zz - I_yy)qr]
        //   [-I_xz  I_zz] [dr] = [N_ext + I_xz·qr - (I_yy - I_xx)pq]

        const double Ixz = params_.mass.Ixz;
        const double p = state.p;
        const double q = state.q;
        const double r = state.r;

        // 右辺の計算（既知項）
        double RHS_roll = Mx_total + Ixz * p * q - (Izz - Iyy) * q * r;
        double RHS_yaw = Mz_total - Ixz * q * r - (Iyy - Ixx) * p * q;

        // 2x2連立方程式を解く（Cramerの公式）
        double det = Ixx * Izz - Ixz * Ixz;
        dxdt(9) = (Izz * RHS_roll + Ixz * RHS_yaw) / det;   // dp/dt
        dxdt(11) = (Ixz * RHS_roll + Ixx * RHS_yaw) / det;  // dr/dt

        // ピッチ角加速度 (dq/dt)（独立方程式）
        // I_yy·dq + (I_xx - I_zz)pr + I_xz(p² - r²) = M_ext
        double gyro_pitch = (Ixx - Izz) * p * r;
        double coupling_pitch = Ixz * (p * p - r * r);
        dxdt(10) = (My_total - gyro_pitch - coupling_pitch) / Iyy;
        
        // ======== 車輪回転 ========
        
        // 後輪平均回転数からエンジンRPMを計算
        double omega_rear_avg = (state.omega[RL] + state.omega[RR]) / 2.0;
        double engine_rpm = gear_ratios_.wheel_omega_to_engine_rpm(omega_rear_avg, input.gear);
        
        // エンジントルク計算（トルクカーブ参照）
        double engine_torque = engine_map_.get_torque(engine_rpm, input.throttle);
        
        // 駆動トルク（総減速比と効率を考慮）。変速中は遮断（#4）
        double total_ratio = gear_ratios_.get_total_ratio(input.gear);
        double total_drive_torque = (time_ < shift_end_time_)
            ? 0.0
            : engine_torque * total_ratio * params_.powertrain.efficiency;
        // Engine/gearbox inertia reflected to the rear wheels, split evenly (#4).
        // ponytail: kinematic engine (no clutch slip state); add an engine-speed state for launches.
        // During a shift the clutch is open: no torque and no engine inertia on the wheels.
        const double I_reflected_rear = (time_ < shift_end_time_) ? 0.0 : 0.5 * (params_.powertrain.I_engine * total_ratio * total_ratio
            + params_.powertrain.I_gearbox * gear_ratios_.final_ratio * gear_ratios_.final_ratio);
        
        // LSDによる左右トルク配分
        double torque_RL, torque_RR;
        lsd_.distribute_torque(
            total_drive_torque,
            state.omega[RL],
            state.omega[RR],
            torque_RL,
            torque_RR
        );
        
        // ブレーキ最大トルク（ライン圧の1次遅れは step() で ZOH 更新）
        const auto T_brake_max = compute_brake_torque_max(
            params_.brake.pressure_lag_s > 0.0 ? brake_pressure_lagged_ : compute_brake_pressure_cmd(input.brake));

        for (int i = 0; i < 4; ++i) {
            double T_drive = 0.0;
            if (i == RL) {
                T_drive = torque_RL;  // 後輪左
            } else if (i == RR) {
                T_drive = torque_RR;  // 後輪右
            }
            // 前輪（FL, FR）は駆動力なし

            // ブレーキトルク（回転と逆向き）。tanh 平滑化で停止時の符号反転チャタリングを防ぐ (#36)
            // ponytail: viscous near ω=0 (creeps under a net torque), add stick clamp if hill holds matter.
            double T_brake = -T_brake_max[i] * std::tanh(state.omega[i] / params_.brake.omega_smooth);

            // タイヤ座標系の縦力を直接使用（有効転がり半径, #12）
            double T_tire = -tire_Fx_stage[i] * R_eff[i];

            // 転がり抵抗モーメント（#12）
            constexpr double omega_rr = 0.5;  // [rad/s] smoothing through zero speed
            double T_rr = -params_.tire.C_rr * Fz_tire[i] * R_eff[i] * std::tanh(state.omega[i] / omega_rr);

            dxdt(20 + i) = (T_drive + T_brake + T_tire + T_rr) / (wheel_spin_inertia(i) + (i >= RL ? I_reflected_rear : 0.0));
        }
        
        // ======== フレームねじりダイナミクス（Phase A1）========
        // 前後サスペンションによるロールモーメントを計算
        std::array<double, 4> F_susp_total;
        for (int i = 0; i < 4; ++i) {
            F_susp_total[i] = F_spring[i] + F_bw[i] + F_lugre[i] + F_bump[i] + F_jacking[i] + F_arb[i];
        }

        if (rigid_frame) {
            // 剛体モード: phi_twist=dphi_twist=0 を維持（冒頭でクランプ済み）
            dxdt(40) = 0.0;
            dxdt(41) = 0.0;
        } else {
            auto [M_front, M_rear] = compute_suspension_roll_moments(F_susp_total, wheel_pos);

            // フレームねじり運動方程式
            // I_twist · d²φ_twist/dt² = M_front - M_rear - K_chassis·φ_twist - C_chassis·dφ_twist/dt
            double K_chassis_rad = params_.suspension.K_chassis * (180.0 / M_PI);  // Nm/deg → Nm/rad
            double I_twist = params_.suspension.I_twist;
            double C_chassis = params_.suspension.C_chassis;

            double ddphi_twist = (M_front - M_rear
                                 - K_chassis_rad * state.phi_twist
                                 - C_chassis * state.dphi_twist) / I_twist;

            dxdt(40) = state.dphi_twist;
            dxdt(41) = ddphi_twist;
        }

        // ======== キングピン軸トルク（#5）========
        std::array<double, 2> T_kingpin{};
        for (int i = 0; i < 2; ++i) {
            const auto& kc = params_.suspension.kc_corners[i];
            const std::array<double, 6> loads = {
                tire_Fx_stage[i], tire_Fy_stage[i], Fz_tire[i],
                tire_Mx_stage[i], 0.0, tire_Mz_stage[i]};
            double caster = kc.caster;
            const auto& corner_samples = kc.samples;
            if (corner_samples.travel_m.size() >= 2
                && corner_samples.caster_rad.size() == corner_samples.travel_m.size()) {
                caster = lut_eval(corner_samples.travel_m, corner_samples.caster_rad,
                                  susp_travel_from_eq_arr[i], caster);
            }
            for (int k = 0; k < 6; ++k) {
                caster += kc.compliance.caster_per_load[k] * loads[k];  // #7
            }
            T_kingpin[i] = compute_kingpin_torque(
                tire_Mz_stage[i], tire_Fx_stage[i], tire_Fy_stage[i],
                kc.mechanical_trail, caster, kc.scrub_radius,
                std::copysign(1.0, wheel_pos[i].y())) + T_gyro_kingpin[i];
        }
        if (update_cache) kingpin_torque_ = T_kingpin;

        // ======== ステアリングホイールダイナミクス（Phase A2）========
        if (params_.steering.torque_mode) {
            // トルクモード: ステアリング運動方程式を解く

            // 前輪キングピン軸トルク合計（Mz + 機械トレール + スクラブ）
            // Signed: positive for a left turn and negative for a right turn.
            // compute_steering_acceleration adds it as restoring torque.
            double T_SAT_total = T_kingpin[0] + T_kingpin[1];

            // ドライバー入力トルク（input.steeringをトルクとして解釈）
            double T_driver = input.steering;

            // トルク飽和
            T_driver = std::clamp(T_driver, -params_.steering.T_driver_max, params_.steering.T_driver_max);

            // ステアリングホイール角加速度
            double ddelta_sw_accel = compute_steering_acceleration(
                params_.steering.I_steer,
                T_driver,
                T_SAT_total,
                params_.steering.R_steer,
                state.delta_sw,
                state.ddelta_sw,
                params_.steering.T_fric,
                params_.steering.C_steer_torque,
                params_.steering.K_return
            );

            dxdt(42) = state.ddelta_sw;
            dxdt(43) = ddelta_sw_accel;
        } else {
            // 角度モード（後方互換性）: ステアリングホイール角は使用しない
            // 状態変数は存在するが、微分はゼロ
            dxdt(42) = 0.0;
            dxdt(43) = 0.0;
        }

        return dxdt;
    }
};

} // namespace fsae::vehicle
