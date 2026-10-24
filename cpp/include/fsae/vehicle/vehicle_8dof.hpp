#pragma once
/// @file vehicle_8dof.hpp
/// @brief 8自由度ビークルモデル（6DOF + 4輪ばね下垂直動）
///
/// 6DOFモデルをベースに、各輪のばね下質量の垂直動を追加：
/// - ばね上: x, y, z, φ, θ, ψ（6DOF）
/// - ばね下: z_u[4]（4DOF）
///
/// 設計書参照: docs/設計書_統合版.md セクション3

#include "vehicle_params.hpp"
#include "vehicle_state.hpp"
#include "fsae/physics/tire/tmeasy.hpp"
#include "fsae/math/rotation.hpp"

#include <cmath>
#include <algorithm>
#include <array>

namespace fsae::vehicle {

/// 8DOF車両モデル
class Vehicle8DOF {
public:
    using Params = VehicleParams;
    
    /// 8DOF状態（32状態変数）
    struct State8DOF {
        // ばね上位置（慣性座標系）
        double x = 0, y = 0, z = 0;
        double phi = 0, theta = 0, psi = 0;
        
        // ばね上速度（車体座標系）
        double vx = 0, vy = 0, vz = 0;
        double p = 0, q = 0, r = 0;
        
        // ばね下垂直位置・速度（各輪）
        std::array<double, 4> z_u = {0, 0, 0, 0};
        std::array<double, 4> vz_u = {0, 0, 0, 0};
        
        // 車輪回転速度
        std::array<double, 4> omega = {0, 0, 0, 0};
        
        // タイヤ変形量（TMeasy）
        std::array<double, 4> tire_xi = {0, 0, 0, 0};
        std::array<double, 4> tire_eta = {0, 0, 0, 0};
        
        double speed() const { return std::sqrt(vx*vx + vy*vy); }
        
        // 状態次元: 6(位置) + 6(速度) + 8(ばね下) + 4(車輪) + 8(タイヤ) = 32
        static constexpr int SIZE = 32;
    };
    
    explicit Vehicle8DOF(const Params& params) : params_(params) {
        setup_tire_params();
        compute_equilibrium();
    }
    
    const State8DOF& state() const { return state_; }
    State8DOF& state() { return state_; }
    const Params& params() const { return params_; }
    double time() const { return time_; }
    double z_equilibrium() const { return z_equilibrium_; }
    
    void set_initial_velocity(double vx) {
        state_.vx = vx;
        for (int i = 0; i < 4; ++i) {
            state_.omega[i] = vx / params_.tire.radius;
        }
    }
    
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
    }
    
    void reset() {
        state_ = State8DOF{};
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
        }
        time_ = 0;
    }

private:
    Params params_;
    State8DOF state_;
    double time_ = 0;
    double z_equilibrium_ = 0;
    std::array<double, 4> z_u_equilibrium_ = {0, 0, 0, 0};
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
        tire_params_.Cy = 60000.0;
    }
    
    void compute_equilibrium() {
        double m = params_.mass.m_sprung;
        double g = 9.81;
        double a = params_.mass.a;
        double b = params_.mass.b;
        double L = params_.mass.wheelbase;
        
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
        
        // ばね下平衡位置（タイヤ剛性で支える）
        double k_tire = params_.tire.k_tire;
        for (int i = 0; i < 4; ++i) {
            double total_load = static_Fz_[i] + params_.mass.m_unsprung[i] * g;
            z_u_equilibrium_[i] = total_load / k_tire;
        }
        
        // 初期状態を設定
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
        }
    }
    
    std::array<Eigen::Vector3d, 4> get_wheel_positions() const {
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double tf = params_.mass.track_f / 2;
        const double tr = params_.mass.track_r / 2;
        
        return {{
            Eigen::Vector3d(a, -tf, 0),
            Eigen::Vector3d(a, tf, 0),
            Eigen::Vector3d(-b, -tr, 0),
            Eigen::Vector3d(-b, tr, 0)
        }};
    }
    
    Eigen::VectorXd state_to_vector(const State8DOF& s) const {
        Eigen::VectorXd x(State8DOF::SIZE);
        x(0) = s.x; x(1) = s.y; x(2) = s.z;
        x(3) = s.phi; x(4) = s.theta; x(5) = s.psi;
        x(6) = s.vx; x(7) = s.vy; x(8) = s.vz;
        x(9) = s.p; x(10) = s.q; x(11) = s.r;
        for (int i = 0; i < 4; ++i) {
            x(12 + i) = s.z_u[i];
            x(16 + i) = s.vz_u[i];
            x(20 + i) = s.omega[i];
            x(24 + i) = s.tire_xi[i];
            x(28 + i) = s.tire_eta[i];
        }
        return x;
    }
    
    State8DOF vector_to_state(const Eigen::VectorXd& x) const {
        State8DOF s;
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
        }
        return s;
    }
    
    Eigen::VectorXd compute_derivatives(const State8DOF& state, const DriverInput& input) {
        Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(State8DOF::SIZE);
        
        const double m = params_.mass.m_sprung;
        const double g = 9.81;
        const double Ixx = params_.mass.Ixx;
        const double Iyy = params_.mass.Iyy;
        const double Izz = params_.mass.Izz;
        const double k_tire = params_.tire.k_tire;
        const double c_tire = params_.tire.c_tire;
        
        auto wheel_pos = get_wheel_positions();
        
        // ========== 位置の微分 ==========
        math::EulerAngles<double> euler{state.phi, state.theta, state.psi};
        auto R = math::euler_to_rotation_matrix(euler);
        
        Eigen::Vector3d v_body(state.vx, state.vy, state.vz);
        Eigen::Vector3d v_inertial = R.transpose() * v_body;
        dxdt(0) = v_inertial(0);
        dxdt(1) = v_inertial(1);
        dxdt(2) = v_inertial(2);
        
        auto E = math::angular_velocity_to_euler_rate(euler);
        Eigen::Vector3d omega_body(state.p, state.q, state.r);
        Eigen::Vector3d euler_rate = E * omega_body;
        dxdt(3) = euler_rate(0);
        dxdt(4) = euler_rate(1);
        dxdt(5) = euler_rate(2);
        
        // ========== 各輪の力計算 ==========
        std::array<double, 4> F_spring, F_damper, Fz_tire;
        std::array<double, 4> Fx_wheel, Fy_wheel;
        
        double delta = input.steering;
        std::array<double, 4> steer = {delta, delta, 0, 0};
        
        for (int i = 0; i < 4; ++i) {
            // ばね上の車輪位置でのz座標
            double z_sprung_at_wheel = state.z 
                - wheel_pos[i].x() * std::sin(state.theta)
                + wheel_pos[i].y() * std::sin(state.phi);
            
            // サスペンション変位 = ばね上 - ばね下
            double susp_travel = z_sprung_at_wheel - state.z_u[i];
            
            // サスペンション速度
            double vz_sprung_at_wheel = state.vz
                - wheel_pos[i].x() * state.q * std::cos(state.theta)
                + wheel_pos[i].y() * state.p * std::cos(state.phi);
            double susp_velocity = vz_sprung_at_wheel - state.vz_u[i];
            
            // スプリング・ダンパー力
            double mr = params_.suspension.motion_ratio[i];
            double k_eff = params_.suspension.k_spring[i] * mr * mr;
            double c_eff = params_.suspension.c0 * mr * mr;
            
            // 平衡からの偏差に対する力
            double susp_travel_from_eq = susp_travel - (z_equilibrium_ - z_u_equilibrium_[i]);
            F_spring[i] = static_Fz_[i] + k_eff * susp_travel_from_eq;
            F_damper[i] = c_eff * susp_velocity;
            
            // タイヤ垂直力（タイヤ剛性モデル）
            double tire_deflection = state.z_u[i] - z_u_equilibrium_[i];
            double tire_velocity = state.vz_u[i];
            Fz_tire[i] = static_Fz_[i] + params_.mass.m_unsprung[i] * g 
                       + k_tire * tire_deflection + c_tire * tire_velocity;
            Fz_tire[i] = std::max(Fz_tire[i], 0.0);
            
            // ========== タイヤ力計算 ==========
            double vx_contact = state.vx - state.r * wheel_pos[i].y();
            double vy_contact = state.vy + state.r * wheel_pos[i].x();
            
            double c = std::cos(steer[i]), s = std::sin(steer[i]);
            double vx_wheel = c * vx_contact + s * vy_contact;
            double vy_wheel = -s * vx_contact + c * vy_contact;
            
            double omega_r = state.omega[i] * params_.tire.radius;
            double vsx = vx_wheel - omega_r;
            double vsy = vy_wheel;
            double vcx = std::abs(omega_r);
            
            physics::tire::TMeasyState<double> tire_state{state.tire_xi[i], state.tire_eta[i]};
            auto tire_out = physics::tire::compute_tmeasy(
                tire_params_, tire_state, Fz_tire[i], vsx, vsy, std::max(vcx, 0.1));
            
            Fx_wheel[i] = c * tire_out.Fx - s * tire_out.Fy;
            Fy_wheel[i] = s * tire_out.Fx + c * tire_out.Fy;
            
            // タイヤ変形量の微分
            dxdt(24 + i) = vsx - (vcx / params_.tire.Lx) * state.tire_xi[i];
            dxdt(28 + i) = vsy - (vcx / params_.tire.Ly) * state.tire_eta[i];
            
            // ========== ばね下運動方程式 (SAE Z-down: 重力+ばね力-タイヤ反力) ==========
            dxdt(12 + i) = state.vz_u[i];  // dz_u/dt
            
            double m_u = params_.mass.m_unsprung[i];
            double F_susp_on_unsprung = F_spring[i] + F_damper[i];
            double a_z_u = g + (F_susp_on_unsprung - Fz_tire[i]) / m_u;
            dxdt(16 + i) = a_z_u;  // dvz_u/dt
        }
        
        // ========== ばね上の力とモーメント ==========
        double Fx_total = 0, Fy_total = 0, Fz_total = 0;
        double Mx_total = 0, My_total = 0, Mz_total = 0;
        
        const double h_cg = params_.mass.h_cg;  // 重心高
        
        for (int i = 0; i < 4; ++i) {
            Fx_total += Fx_wheel[i];
            Fy_total += Fy_wheel[i];
            
            // サスペンション反力（ばね上への力）
            double F_susp = F_spring[i] + F_damper[i];
            Fz_total -= F_susp;
            
            // ロールモーメント = サスペンション力による成分 + タイヤ横力×重心高
            Mx_total -= wheel_pos[i].y() * F_susp;
            Mx_total += Fy_wheel[i] * h_cg;  // 横力による重心周りのロールモーメント
            
            My_total += wheel_pos[i].x() * F_susp;
            Mz_total += wheel_pos[i].x() * Fy_wheel[i] - wheel_pos[i].y() * Fx_wheel[i];
        }
        
        // 空力
        double v2 = state.vx * state.vx + state.vy * state.vy;
        Fz_total += 0.5 * params_.aero.rho * v2 * params_.aero.CL_A;
        Fx_total -= 0.5 * params_.aero.rho * v2 * params_.aero.CD_A;
        
        // 重力
        Eigen::Vector3d g_body = math::gravity_in_body(euler, g);
        
        // ========== ばね上並進運動 ==========
        dxdt(6) = Fx_total / m + g_body(0) + state.r * state.vy - state.q * state.vz;
        dxdt(7) = Fy_total / m + g_body(1) - state.r * state.vx + state.p * state.vz;
        dxdt(8) = Fz_total / m + g_body(2) + state.q * state.vx - state.p * state.vy;
        
        // ========== ばね上回転運動 ==========
        dxdt(9) = Mx_total / Ixx;
        dxdt(10) = My_total / Iyy;
        dxdt(11) = Mz_total / Izz;
        
        // ========== 車輪回転 ==========
        double Iw = params_.mass.Iw;
        double R_tire = params_.tire.radius;
        
        double engine_torque = input.throttle * params_.powertrain.max_torque;
        double drive_torque = engine_torque * params_.powertrain.gear_ratio * params_.powertrain.efficiency / 2;
        
        for (int i = 0; i < 4; ++i) {
            double T_drive = (i >= 2) ? drive_torque : 0;
            
            double c = std::cos(steer[i]), s = std::sin(steer[i]);
            double Fx_w = c * Fx_wheel[i] + s * Fy_wheel[i];
            double T_tire = -Fx_w * R_tire;
            
            dxdt(20 + i) = (T_drive + T_tire) / Iw;
        }
        
        return dxdt;
    }
};

} // namespace fsae::vehicle
