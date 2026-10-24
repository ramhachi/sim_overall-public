#pragma once
/// @file vehicle_3dof.hpp
/// @brief 3自由度プレーナービークルモデル（デバッグ用簡略版）
///
/// 平面運動のみ（x, y, ψ）に限定した車両モデル。
/// - 垂直運動なし（z = 0固定）
/// - ロール/ピッチなし（φ = θ = 0固定）
/// - タイヤは静的荷重のみ

#include "vehicle_params.hpp"
#include "vehicle_state.hpp"
#include "fsae/physics/tire/tmeasy.hpp"

#include <cmath>
#include <algorithm>
#include <array>

namespace fsae::vehicle {

/// 3DOF簡略化ビークルモデル
class Vehicle3DOF {
public:
    using Params = VehicleParams;
    
    /// 3DOF状態
    struct State3DOF {
        // 位置（慣性座標系）
        double x = 0;
        double y = 0;
        double psi = 0;     // ヨー角 [rad]
        
        // 速度（車体座標系）
        double vx = 0;      // 縦速度 [m/s]
        double vy = 0;      // 横速度 [m/s]
        double r = 0;       // ヨーレート [rad/s]
        
        // 車輪回転速度
        std::array<double, 4> omega = {0, 0, 0, 0};
        
        // タイヤ変形量（TMeasy）
        std::array<double, 4> tire_xi = {0, 0, 0, 0};
        std::array<double, 4> tire_eta = {0, 0, 0, 0};
        
        double speed() const { return std::sqrt(vx*vx + vy*vy); }
    };
    
    explicit Vehicle3DOF(const Params& params) : params_(params) {
        setup_tire_params();
        compute_static_loads();
    }
    
    const State3DOF& state() const { return state_; }
    State3DOF& state() { return state_; }
    const Params& params() const { return params_; }
    double time() const { return time_; }
    
    /// 初期速度を設定
    void set_initial_velocity(double vx) {
        state_.vx = vx;
        for (int i = 0; i < 4; ++i) {
            state_.omega[i] = vx / params_.tire.radius;
        }
    }
    
    /// RK4で1ステップ進める
    void step(double dt, const DriverInput& input) {
        // 状態をベクトルに変換
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
    }
    
    void reset() {
        state_ = State3DOF{};
        time_ = 0;
    }

private:
    Params params_;
    State3DOF state_;
    double time_ = 0;
    std::array<double, 4> static_Fz_ = {0, 0, 0, 0};
    physics::tire::TMeasyParams<double> tire_params_;
    
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
    }
    
    void compute_static_loads() {
        double m = params_.mass.m_sprung;
        double g = 9.81;
        double a = params_.mass.a;
        double b = params_.mass.b;
        double L = params_.mass.wheelbase;
        double tf = params_.mass.track_f / 2;
        double tr = params_.mass.track_r / 2;
        
        // 静的荷重配分
        double W = m * g;
        double Wf = W * b / L;  // フロント荷重
        double Wr = W * a / L;  // リア荷重
        
        static_Fz_[FL] = Wf / 2;
        static_Fz_[FR] = Wf / 2;
        static_Fz_[RL] = Wr / 2;
        static_Fz_[RR] = Wr / 2;
    }
    
    Eigen::VectorXd state_to_vector(const State3DOF& s) const {
        // 3 + 3 + 4 + 8 = 18
        Eigen::VectorXd x(18);
        x(0) = s.x; x(1) = s.y; x(2) = s.psi;
        x(3) = s.vx; x(4) = s.vy; x(5) = s.r;
        for (int i = 0; i < 4; ++i) {
            x(6 + i) = s.omega[i];
            x(10 + i) = s.tire_xi[i];
            x(14 + i) = s.tire_eta[i];
        }
        return x;
    }
    
    State3DOF vector_to_state(const Eigen::VectorXd& x) const {
        State3DOF s;
        s.x = x(0); s.y = x(1); s.psi = x(2);
        s.vx = x(3); s.vy = x(4); s.r = x(5);
        for (int i = 0; i < 4; ++i) {
            s.omega[i] = x(6 + i);
            s.tire_xi[i] = x(10 + i);
            s.tire_eta[i] = x(14 + i);
        }
        return s;
    }
    
    Eigen::VectorXd compute_derivatives(const State3DOF& state, const DriverInput& input) {
        Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(18);
        
        // 車輪位置
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double tf = params_.mass.track_f / 2;
        const double tr = params_.mass.track_r / 2;
        
        std::array<double, 4> x_pos = {a, a, -b, -b};
        std::array<double, 4> y_pos = {-tf, tf, -tr, tr};
        
        // ステアリング角（Ackermann簡略化）
        double delta = input.steering;
        std::array<double, 4> steer = {delta, delta, 0, 0};  // 簡略：前輪同角
        
        // 各輪のタイヤ力計算
        std::array<double, 4> Fx_wheel, Fy_wheel;
        
        for (int i = 0; i < 4; ++i) {
            // 接地点速度（車両座標系）
            double vx_contact = state.vx - state.r * y_pos[i];
            double vy_contact = state.vy + state.r * x_pos[i];
            
            // ホイール座標系に変換
            double c = std::cos(steer[i]), s = std::sin(steer[i]);
            double vx_wheel = c * vx_contact + s * vy_contact;
            double vy_wheel = -s * vx_contact + c * vy_contact;
            
            // スリップ速度
            double omega_r = state.omega[i] * params_.tire.radius;
            double vsx = vx_wheel - omega_r;  // 縦スリップ速度
            double vsy = vy_wheel;            // 横スリップ速度
            double vcx = std::abs(omega_r);   // 転がり速度
            
            // TMeasy力計算
            physics::tire::TMeasyState<double> tire_state{state.tire_xi[i], state.tire_eta[i]};
            auto tire_out = physics::tire::compute_tmeasy(
                tire_params_, tire_state, static_Fz_[i], vsx, vsy, std::max(vcx, 0.1));
            
            // ホイール→車両座標変換
            Fx_wheel[i] = c * tire_out.Fx - s * tire_out.Fy;
            Fy_wheel[i] = s * tire_out.Fx + c * tire_out.Fy;
            
            // タイヤ変形量の微分
            double v_eff = std::sqrt(vcx*vcx + params_.tire.v_N*params_.tire.v_N);
            dxdt(10 + i) = vsx - (vcx / params_.tire.Lx) * state.tire_xi[i];
            dxdt(14 + i) = vsy - (vcx / params_.tire.Ly) * state.tire_eta[i];
        }
        
        // 合計力とモーメント
        double Fx_total = 0, Fy_total = 0, Mz_total = 0;
        for (int i = 0; i < 4; ++i) {
            Fx_total += Fx_wheel[i];
            Fy_total += Fy_wheel[i];
            Mz_total += x_pos[i] * Fy_wheel[i] - y_pos[i] * Fx_wheel[i];
        }
        
        // 空力抵抗
        double v2 = state.vx * state.vx + state.vy * state.vy;
        Fx_total -= 0.5 * params_.aero.rho * v2 * params_.aero.CD_A;
        
        // 運動方程式
        double m = params_.mass.m_sprung;
        double Izz = params_.mass.Izz;
        
        dxdt(3) = Fx_total / m + state.r * state.vy;   // dvx/dt
        dxdt(4) = Fy_total / m - state.r * state.vx;   // dvy/dt
        dxdt(5) = Mz_total / Izz;                      // dr/dt
        
        // 位置の微分（慣性座標系）
        double c_psi = std::cos(state.psi), s_psi = std::sin(state.psi);
        dxdt(0) = c_psi * state.vx - s_psi * state.vy;  // dx/dt
        dxdt(1) = s_psi * state.vx + c_psi * state.vy;  // dy/dt
        dxdt(2) = state.r;                               // dpsi/dt
        
        // 車輪回転の微分
        double Iw = params_.mass.Iw;
        double R = params_.tire.radius;
        
        // 駆動トルク（後輪駆動）
        double engine_torque = input.throttle * params_.powertrain.max_torque;
        double drive_torque = engine_torque * params_.powertrain.gear_ratio * params_.powertrain.efficiency / 2;
        
        for (int i = 0; i < 4; ++i) {
            double T_drive = (i >= 2) ? drive_torque : 0;  // 後輪のみ
            
            // ホイール座標系でのFx（駆動反力）
            double c = std::cos(steer[i]), s = std::sin(steer[i]);
            double Fx_w = c * Fx_wheel[i] + s * Fy_wheel[i];  // 逆変換
            double T_tire = -Fx_w * R;
            
            dxdt(6 + i) = (T_drive + T_tire) / Iw;
        }
        
        return dxdt;
    }
};

} // namespace fsae::vehicle
