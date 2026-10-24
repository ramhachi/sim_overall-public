#pragma once
/// @file vehicle_6dof.hpp
/// @brief 6自由度ビークルモデル（3DOF + ロール/ピッチ/ヒーブ）
///
/// 3DOFプレーナーモデルをベースに、垂直方向ダイナミクスを追加：
/// - 平面運動: x, y, ψ（ヨー）
/// - 垂直運動: z（ヒーブ）、φ（ロール）、θ（ピッチ）
///
/// 設計書参照: docs/設計書_統合版.md セクション3
///
/// スクリーニング専用（Issue #30）: FastLap（dt 10 ms）で GA 初期世代・感度スクリーニングに使う。
/// 最終評価は 14DOF。ばね下・ARB・K&C・ねじり・熱・DRS・SAT は省略。空力は 14DOF と同じ
/// AeroMap4D（ride height・roll・yaw 依存、モーメント込み）、タイヤは荷重感度 dmu_dFz 付き TMeasy、
/// タイヤたわみは RK4 外で半陰的（後退 Euler）に更新するので低速/高速で stiff にならない。
/// docs/performance_lightweight_sim_plan_2026-06-15.md の「準静的 Fz＋表タイヤ」案は未実装。

#include "vehicle_params.hpp"
#include "vehicle_state.hpp"
#include "fsae/physics/tire/tmeasy.hpp"
#include "fsae/physics/aero/aero_map_4d.hpp"
#include "fsae/math/rotation.hpp"

#include <cmath>
#include <algorithm>
#include <array>
#include <cctype>
#include <iostream>
#include <string>

namespace fsae::vehicle {

/// 6DOF車両モデル
class Vehicle6DOF {
public:
    using Params = VehicleParams;
    
    /// 6DOF状態（24状態変数）
    struct State6DOF {
        // 位置（慣性座標系）
        double x = 0, y = 0, z = 0;
        double phi = 0, theta = 0, psi = 0;  // ロール、ピッチ、ヨー [rad]
        
        // 速度（車体座標系）
        double vx = 0, vy = 0, vz = 0;
        double p = 0, q = 0, r = 0;  // 角速度 [rad/s]
        
        // 車輪回転速度
        std::array<double, 4> omega = {0, 0, 0, 0};
        
        // タイヤ変形量（TMeasy）
        std::array<double, 4> tire_xi = {0, 0, 0, 0};
        std::array<double, 4> tire_eta = {0, 0, 0, 0};
        
        double speed() const { return std::sqrt(vx*vx + vy*vy); }
        
        // 状態次元: 6(位置) + 6(速度) + 4(車輪) + 8(タイヤ) = 24
        static constexpr int SIZE = 24;
    };
    
    explicit Vehicle6DOF(const Params& params) : params_(params) {
        setup_tire_params();
        setup_aero_map();
        compute_equilibrium();
    }
    
    const State6DOF& state() const { return state_; }
    State6DOF& state() { return state_; }
    const Params& params() const { return params_; }
    double time() const { return time_; }
    
    /// 初期速度を設定（平衡状態を維持）
    void set_initial_velocity(double vx) {
        state_.vx = vx;
        for (int i = 0; i < 4; ++i) {
            state_.omega[i] = vx / params_.tire.radius;
        }
    }
    
    /// RK4で1ステップ進める
    void step(double dt, const DriverInput& input) {
        Eigen::VectorXd x = state_to_vector(state_);
        
        auto f = [this, &input](const Eigen::VectorXd& x_) {
            return compute_derivatives(vector_to_state(x_), input);
        };
        
        Eigen::VectorXd k1 = f(x);
        Eigen::VectorXd k2 = f(x + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(x + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(x + dt * k3);
        
        x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        state_ = vector_to_state(x);
        time_ += dt;

        // タイヤたわみ: dξ/dt = v_s − (v_c/L) ξ を後退 Euler で更新（無条件安定、Issue #30）
        const double delta = input.steering;
        const auto wheel_pos = get_wheel_positions();
        for (int i = 0; i < 4; ++i) {
            const auto sl = wheel_slip(state_, wheel_pos[i], i < 2 ? delta : 0.0, i);
            state_.tire_xi[i] = (state_.tire_xi[i] + dt * sl.vsx) / (1.0 + dt * sl.vcx / params_.tire.Lx);
            state_.tire_eta[i] = (state_.tire_eta[i] + dt * sl.vsy) / (1.0 + dt * sl.vcx / params_.tire.Ly);
        }
    }
    
    void reset() {
        state_ = State6DOF{};
        state_.z = z_equilibrium_;
        time_ = 0;
    }
    
    /// 平衡状態のz座標を取得
    double z_equilibrium() const { return z_equilibrium_; }

private:
    Params params_;
    State6DOF state_;
    double time_ = 0;
    double z_equilibrium_ = 0;  // 静的平衡位置
    std::array<double, 4> static_Fz_ = {0, 0, 0, 0};  // 静的荷重
    physics::tire::TMeasyParams<double> tire_params_;
    physics::aero::AeroMap4D<double> aero_map_;

    struct WheelSlip { double vsx, vsy, vcx, c, s; };

    /// 接地点スリップ速度（ホイール座標）
    WheelSlip wheel_slip(const State6DOF& state, const Eigen::Vector3d& pos, double steer, int i) const {
        const double vx_contact = state.vx - state.r * pos.y();
        const double vy_contact = state.vy + state.r * pos.x();
        const double c = std::cos(steer), s = std::sin(steer);
        const double vx_wheel = c * vx_contact + s * vy_contact;
        const double vy_wheel = -s * vx_contact + c * vy_contact;
        const double omega_r = state.omega[i] * params_.tire.radius;
        return {vx_wheel - omega_r, vy_wheel, std::abs(omega_r), c, s};
    }

    /// 14DOF と同じ AeroMap4D 設定（vehicle_14dof_new.hpp setup_aero_map と同一）
    void setup_aero_map() {
        aero_map_.params().rho = params_.aero.rho;
        aero_map_.set_reference_conditions(params_.aero.reference_speed, params_.aero.reference_density);
        aero_map_.set_coefficient_defaults(-std::abs(params_.aero.CL_A),
                                           std::max(params_.aero.CD_A, 0.01),
                                           params_.aero.aero_balance);
        std::string mode = params_.aero.map_mode;
        for (auto& ch : mode) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        if (mode == "direct_load" || mode == "force_map" || mode == "load_map") {
            aero_map_.set_direct_load_defaults(
                params_.aero.Fx_ref_total, params_.aero.Fy_ref_total, params_.aero.Fz_ref_total,
                params_.aero.Mx_ref_total, params_.aero.My_ref_total, params_.aero.Mz_ref_total);
        }
        if (!params_.aero.aero_map_file.empty() && !aero_map_.load_from_json(params_.aero.aero_map_file)) {
            std::cerr << "Failed to load aero map: " << params_.aero.aero_map_file
                      << " (falling back to defaults)" << std::endl;
        }
    }

    static constexpr int FL = 0, FR = 1, RL = 2, RR = 3;
    
    void setup_tire_params() {
        tire_params_.mu_x_max = params_.tire.mu_x_max;
        tire_params_.mu_y_max = params_.tire.mu_y_max;
        tire_params_.Fz_nom = params_.tire.Fz_nom;
        tire_params_.Lx = params_.tire.Lx;
        tire_params_.Ly = params_.tire.Ly;
        tire_params_.v_N = params_.tire.v_N;
        tire_params_.Cx = 80000.0;
        tire_params_.Cy = 22000.0;  // 実測ベース
        tire_params_.dmu_dFz = params_.tire.dmu_dFz;  // 荷重感度（14DOF と同じ）
    }
    
    /// 静的平衡状態を計算
    void compute_equilibrium() {
        double m = params_.mass.m_sprung;
        double g = 9.81;
        double a = params_.mass.a;
        double b = params_.mass.b;
        double L = params_.mass.wheelbase;
        
        // 静的荷重配分
        double W = m * g;
        double Wf = W * b / L;
        double Wr = W * a / L;
        
        static_Fz_[FL] = Wf / 2;
        static_Fz_[FR] = Wf / 2;
        static_Fz_[RL] = Wr / 2;
        static_Fz_[RR] = Wr / 2;
        
        // サスペンションの平衡位置を計算
        // F_spring = k * z → z = F / k
        double k_avg = 0;
        for (int i = 0; i < 4; ++i) {
            double mr = params_.suspension.motion_ratio[i];
            k_avg += params_.suspension.k_spring[i] * mr * mr;
        }
        k_avg /= 4;
        
        // 平衡z位置（重力とサスペンション力のバランス）
        z_equilibrium_ = W / (4 * k_avg);
        
        // 初期状態を平衡位置に設定
        state_.z = z_equilibrium_;
    }
    
    /// 車輪位置を取得
    std::array<Eigen::Vector3d, 4> get_wheel_positions() const {
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double tf = params_.mass.track_f / 2;
        const double tr = params_.mass.track_r / 2;
        
        return {{
            Eigen::Vector3d(a, -tf, 0),   // FL
            Eigen::Vector3d(a, tf, 0),    // FR
            Eigen::Vector3d(-b, -tr, 0),  // RL
            Eigen::Vector3d(-b, tr, 0)    // RR
        }};
    }
    
    Eigen::VectorXd state_to_vector(const State6DOF& s) const {
        Eigen::VectorXd x(State6DOF::SIZE);
        x(0) = s.x; x(1) = s.y; x(2) = s.z;
        x(3) = s.phi; x(4) = s.theta; x(5) = s.psi;
        x(6) = s.vx; x(7) = s.vy; x(8) = s.vz;
        x(9) = s.p; x(10) = s.q; x(11) = s.r;
        for (int i = 0; i < 4; ++i) {
            x(12 + i) = s.omega[i];
            x(16 + i) = s.tire_xi[i];
            x(20 + i) = s.tire_eta[i];
        }
        return x;
    }
    
    State6DOF vector_to_state(const Eigen::VectorXd& x) const {
        State6DOF s;
        s.x = x(0); s.y = x(1); s.z = x(2);
        s.phi = x(3); s.theta = x(4); s.psi = x(5);
        s.vx = x(6); s.vy = x(7); s.vz = x(8);
        s.p = x(9); s.q = x(10); s.r = x(11);
        for (int i = 0; i < 4; ++i) {
            s.omega[i] = x(12 + i);
            s.tire_xi[i] = x(16 + i);
            s.tire_eta[i] = x(20 + i);
        }
        return s;
    }
    
    Eigen::VectorXd compute_derivatives(const State6DOF& state, const DriverInput& input) {
        Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(State6DOF::SIZE);
        
        const double m = params_.mass.m_sprung;
        const double g = 9.81;
        const double Ixx = params_.mass.Ixx;
        const double Iyy = params_.mass.Iyy;
        const double Izz = params_.mass.Izz;
        
        auto wheel_pos = get_wheel_positions();
        
        // ========== 位置の微分（慣性座標系への変換） ==========
        math::EulerAngles<double> euler{state.phi, state.theta, state.psi};
        auto R = math::euler_to_rotation_matrix(euler);
        
        // 車体速度を慣性座標系に変換
        Eigen::Vector3d v_body(state.vx, state.vy, state.vz);
        Eigen::Vector3d v_inertial = R.transpose() * v_body;
        dxdt(0) = v_inertial(0);  // dx/dt
        dxdt(1) = v_inertial(1);  // dy/dt
        dxdt(2) = v_inertial(2);  // dz/dt
        
        // オイラー角速度
        auto E = math::angular_velocity_to_euler_rate(euler);
        Eigen::Vector3d omega_body(state.p, state.q, state.r);
        Eigen::Vector3d euler_rate = E * omega_body;
        dxdt(3) = euler_rate(0);  // dphi/dt
        dxdt(4) = euler_rate(1);  // dtheta/dt
        dxdt(5) = euler_rate(2);  // dpsi/dt
        
        // ========== 各輪のサスペンション変位と力 ==========
        std::array<double, 4> susp_travel, susp_velocity, F_spring, F_damper, Fz_tire;
        std::array<double, 4> Fx_wheel, Fy_wheel;
        
        // ステアリング角
        double delta = input.steering;
        std::array<double, 4> steer = {delta, delta, 0, 0};
        
        for (int i = 0; i < 4; ++i) {
            // 車輪位置でのばね上高さ
            // SAE座標系: +z = 下向き、+theta = ノーズアップ、+phi = 右下がり
            // ピッチ: theta > 0 → 前輪が上がる = z減少、後輪が下がる = z増加
            // ロール: phi > 0 → 右輪が下がる = z増加、左輪が上がる = z減少
            double z_sprung_at_wheel = state.z 
                - wheel_pos[i].x() * std::sin(state.theta)  // 符号修正
                + wheel_pos[i].y() * std::sin(state.phi);   // 符号修正
            
            // サスペンション変位（平衡位置からの偏差）
            susp_travel[i] = z_sprung_at_wheel - z_equilibrium_;
            
            // サスペンション速度（同様に符号修正）
            susp_velocity[i] = state.vz
                - wheel_pos[i].x() * state.q * std::cos(state.theta)
                + wheel_pos[i].y() * state.p * std::cos(state.phi);
            
            // スプリング力（静的成分 + 偏差成分）
            // 静的成分: 平衡状態で静的荷重を支える力
            // 偏差成分: 平衡からの変位による追加の力
            double mr = params_.suspension.motion_ratio[i];
            double k_eff = params_.suspension.k_spring[i] * mr * mr;
            double F_spring_static = static_Fz_[i];  // 平衡時の力
            double F_spring_delta = k_eff * susp_travel[i];  // 偏差による力
            F_spring[i] = F_spring_static + F_spring_delta;
            
            // ダンパー力（単純線形減衰）
            double c_eff = params_.suspension.c0 * mr * mr;
            F_damper[i] = c_eff * susp_velocity[i];
            
            // タイヤ垂直荷重 = サスペンション力（路面からの反力）
            Fz_tire[i] = F_spring[i] + F_damper[i];
            Fz_tire[i] = std::max(Fz_tire[i], 0.0);
            
            // ========== タイヤ力計算 ==========
            const auto sl = wheel_slip(state, wheel_pos[i], steer[i], i);
            const double vsx = sl.vsx, vsy = sl.vsy, vcx = sl.vcx, c = sl.c, s = sl.s;
            
            // TMeasy力計算
            physics::tire::TMeasyState<double> tire_state{state.tire_xi[i], state.tire_eta[i]};
            auto tire_out = physics::tire::compute_tmeasy(
                tire_params_, tire_state, Fz_tire[i], vsx, vsy, std::max(vcx, 0.1));
            
            // ホイール→車両座標変換
            Fx_wheel[i] = c * tire_out.Fx - s * tire_out.Fy;
            Fy_wheel[i] = s * tire_out.Fx + c * tire_out.Fy;
            
            // タイヤ変形量は step() で半陰的に更新（RK4 段では凍結）
        }
        
        // ========== 合計力とモーメント ==========
        double Fx_total = 0, Fy_total = 0, Fz_total = 0;
        double Mx_total = 0, My_total = 0, Mz_total = 0;
        
        const double h_cg = params_.mass.h_cg;  // 重心高
        
        for (int i = 0; i < 4; ++i) {
            Fx_total += Fx_wheel[i];
            Fy_total += Fy_wheel[i];
            
            // サスペンション力（ばね上への反力）
            double F_susp = F_spring[i] + F_damper[i];
            Fz_total -= F_susp;  // 上向き力は-Z
            
            // ロールモーメント = サスペンション力による成分 + タイヤ横力×重心高
            Mx_total -= wheel_pos[i].y() * F_susp;
            Mx_total += Fy_wheel[i] * h_cg;  // 横力による重心周りのロールモーメント
            
            My_total += wheel_pos[i].x() * F_susp;  // ピッチモーメント
            Mz_total += wheel_pos[i].x() * Fy_wheel[i] - wheel_pos[i].y() * Fx_wheel[i];
        }
        
        // 空力: 14DOF と同じ AeroMap4D（ride height・roll・yaw 依存、モーメント込み）
        const double v_mag = std::hypot(state.vx, state.vy);
        const double rh_front = params_.aero.nominal_front_ride_height_mm
                              - 0.5 * (susp_travel[FL] + susp_travel[FR]) * 1000.0;
        const double rh_rear = params_.aero.nominal_rear_ride_height_mm
                             - 0.5 * (susp_travel[RL] + susp_travel[RR]) * 1000.0;
        const double yaw = v_mag > 0.1 ? std::atan2(state.vy, state.vx) : 0.0;
        const auto aero = aero_map_.compute_forces(v_mag, rh_front, rh_rear, state.phi, yaw, false);
        Fx_total += aero.Fx;  // ドラッグ（負）
        Fy_total += aero.Fy;
        Fz_total -= aero.Fz;  // 負のリフト = 下向き +Z
        Mx_total += aero.Mx;
        My_total += aero.My;
        Mz_total += aero.Mz;

        // 重力（車体座標系）
        Eigen::Vector3d g_body = math::gravity_in_body(euler, g);
        
        // ========== 並進運動方程式 ==========
        dxdt(6) = Fx_total / m + g_body(0) + state.r * state.vy - state.q * state.vz;  // dvx/dt
        dxdt(7) = Fy_total / m + g_body(1) - state.r * state.vx + state.p * state.vz;  // dvy/dt
        dxdt(8) = Fz_total / m + g_body(2) + state.q * state.vx - state.p * state.vy;  // dvz/dt
        
        // ========== 回転運動方程式（簡略化） ==========
        dxdt(9) = Mx_total / Ixx;   // dp/dt
        dxdt(10) = My_total / Iyy;  // dq/dt
        dxdt(11) = Mz_total / Izz;  // dr/dt
        
        // ========== 車輪回転の微分 ==========
        double Iw = params_.mass.Iw;
        double R_tire = params_.tire.radius;
        
        double engine_torque = input.throttle * params_.powertrain.max_torque;
        double drive_torque = engine_torque * params_.powertrain.gear_ratio * params_.powertrain.efficiency / 2;
        
        for (int i = 0; i < 4; ++i) {
            double T_drive = (i >= 2) ? drive_torque : 0;
            
            double c = std::cos(steer[i]), s = std::sin(steer[i]);
            double Fx_w = c * Fx_wheel[i] + s * Fy_wheel[i];
            double T_tire = -Fx_w * R_tire;
            
            dxdt(12 + i) = (T_drive + T_tire) / Iw;
        }
        
        return dxdt;
    }
};

} // namespace fsae::vehicle
