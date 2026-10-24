#pragma once
/// @file vehicle_8dof_full.hpp
/// @brief 8DOF + ARB + Bouc-Wen + LuGre 統合モデル
///
/// 全サスペンション非線形モデルを統合:
/// - Bouc-Wen: ダンパーヒステリシス
/// - LuGre: 摺動部摩擦（スティック・スリップ）
///
/// 状態変数: 36 (8DOF_BW) + 4 (friction_z) = 40

#include "vehicle_8dof_bw.hpp"
#include <fsae/physics/suspension/lugre.hpp>

namespace fsae::vehicle {

/// 8DOF + 全非線形サスペンション状態（40状態変数）
struct State8DOF_Full {
    // 位置（慣性座標系）
    double x = 0, y = 0, z = 0;
    double phi = 0, theta = 0, psi = 0;
    
    // 速度（車体座標系）
    double vx = 0, vy = 0, vz = 0;
    double p = 0, q = 0, r = 0;
    
    // ばね下状態
    std::array<double, 4> z_u = {0, 0, 0, 0};
    std::array<double, 4> vz_u = {0, 0, 0, 0};
    
    // 車輪回転
    std::array<double, 4> omega = {0, 0, 0, 0};
    
    // タイヤ状態（TMeasy用）
    std::array<double, 4> tire_xi = {0, 0, 0, 0};
    std::array<double, 4> tire_eta = {0, 0, 0, 0};
    
    // Bouc-Wenダンパー履歴変数
    std::array<double, 4> damper_z = {0, 0, 0, 0};
    
    // LuGre摩擦毛変形量
    std::array<double, 4> friction_z = {0, 0, 0, 0};
    
    static constexpr int SIZE = 40;  // 36 + 4
};

/// 8DOF + ARB + Bouc-Wen + LuGre 車両モデル
class Vehicle8DOF_Full {
public:
    using Params = VehicleParams;
    
    explicit Vehicle8DOF_Full(const Params& params) : params_(params) {
        setup_tire_params();
        setup_bouc_wen_params();
        setup_lugre_params();
        compute_equilibrium();
    }
    
    const State8DOF_Full& state() const { return state_; }
    State8DOF_Full& state() { return state_; }
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
        
        // RK4積分
        Eigen::VectorXd k1 = f(x);
        Eigen::VectorXd k2 = f(x + 0.5 * dt * k1);
        Eigen::VectorXd k3 = f(x + 0.5 * dt * k2);
        Eigen::VectorXd k4 = f(x + dt * k3);
        
        x = x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        state_ = vector_to_state(x);
        time_ += dt;
    }
    
    void reset() {
        state_ = State8DOF_Full{};
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
            state_.damper_z[i] = 0;
            state_.friction_z[i] = 0;
        }
        time_ = 0;
    }
    
    /// デバッグ用
    std::pair<double, double> get_arb_forces() const { return {arb_force_front_, arb_force_rear_}; }
    std::array<double, 4> get_bouc_wen_forces() const { return bw_forces_; }
    std::array<double, 4> get_lugre_forces() const { return lugre_forces_; }

private:
    Params params_;
    State8DOF_Full state_;
    double time_ = 0;
    double z_equilibrium_ = 0;
    std::array<double, 4> z_u_equilibrium_ = {0, 0, 0, 0};
    std::array<double, 4> static_Fz_ = {0, 0, 0, 0};
    physics::tire::TMeasyParams<double> tire_params_;
    
    // Bouc-Wenパラメータ
    std::array<physics::suspension::BoucWenParams<double>, 4> bw_params_;
    
    // LuGreパラメータ
    std::array<physics::suspension::LuGreParams<double>, 4> lugre_params_;
    
    // デバッグ用
    mutable double arb_force_front_ = 0;
    mutable double arb_force_rear_ = 0;
    mutable std::array<double, 4> bw_forces_ = {0, 0, 0, 0};
    mutable std::array<double, 4> lugre_forces_ = {0, 0, 0, 0};
    
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
        // 設計書セクション5.4に基づく
        for (int i = 0; i < 4; ++i) {
            lugre_params_[i].sigma_0 = params_.suspension.lugre_sigma0;  // 毛剛性 [N/m]
            lugre_params_[i].sigma_1 = params_.suspension.lugre_sigma1;  // 毛減衰 [Ns/m]
            lugre_params_[i].sigma_2 = params_.suspension.lugre_sigma2;  // 粘性摩擦 [Ns/m]
            lugre_params_[i].Fc = params_.suspension.lugre_Fc;           // クーロン摩擦 [N]
            lugre_params_[i].Fs = params_.suspension.lugre_Fs;           // 静止摩擦 [N]
            lugre_params_[i].vs = params_.suspension.lugre_vs;           // ストライベック速度 [m/s]
        }
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
        
        double k_avg = 0;
        for (int i = 0; i < 4; ++i) {
            double mr = params_.suspension.motion_ratio[i];
            k_avg += params_.suspension.k_spring[i] * mr * mr;
        }
        k_avg /= 4;
        z_equilibrium_ = W / (4 * k_avg);
        
        double k_tire = params_.tire.k_tire;
        for (int i = 0; i < 4; ++i) {
            double total_load = static_Fz_[i] + params_.mass.m_unsprung[i] * g;
            z_u_equilibrium_[i] = total_load / k_tire;
        }
        
        state_.z = z_equilibrium_;
        for (int i = 0; i < 4; ++i) {
            state_.z_u[i] = z_u_equilibrium_[i];
        }
    }
    
    Eigen::VectorXd state_to_vector(const State8DOF_Full& s) const {
        Eigen::VectorXd x(State8DOF_Full::SIZE);
        x << s.x, s.y, s.z, s.phi, s.theta, s.psi,
             s.vx, s.vy, s.vz, s.p, s.q, s.r,
             s.z_u[0], s.z_u[1], s.z_u[2], s.z_u[3],
             s.vz_u[0], s.vz_u[1], s.vz_u[2], s.vz_u[3],
             s.omega[0], s.omega[1], s.omega[2], s.omega[3],
             s.tire_xi[0], s.tire_xi[1], s.tire_xi[2], s.tire_xi[3],
             s.tire_eta[0], s.tire_eta[1], s.tire_eta[2], s.tire_eta[3],
             s.damper_z[0], s.damper_z[1], s.damper_z[2], s.damper_z[3],
             s.friction_z[0], s.friction_z[1], s.friction_z[2], s.friction_z[3];
        return x;
    }
    
    State8DOF_Full vector_to_state(const Eigen::VectorXd& x) const {
        State8DOF_Full s;
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
        return s;
    }
    
    std::array<double, 4> compute_arb_forces(const std::array<double, 4>& susp_travel) const {
        std::array<double, 4> F_arb = {0, 0, 0, 0};
        
        double track_f = params_.mass.track_f;
        double roll_diff_f = susp_travel[FL] - susp_travel[FR];
        double roll_angle_f = roll_diff_f / track_f;
        double torque_arb_f = params_.suspension.k_arb_f * roll_angle_f;
        
        double half_track_f = track_f / 2;
        F_arb[FL] = torque_arb_f / half_track_f;
        F_arb[FR] = -torque_arb_f / half_track_f;
        
        double track_r = params_.mass.track_r;
        double roll_diff_r = susp_travel[RL] - susp_travel[RR];
        double roll_angle_r = roll_diff_r / track_r;
        double torque_arb_r = params_.suspension.k_arb_r * roll_angle_r;
        
        double half_track_r = track_r / 2;
        F_arb[RL] = torque_arb_r / half_track_r;
        F_arb[RR] = -torque_arb_r / half_track_r;
        
        arb_force_front_ = torque_arb_f;
        arb_force_rear_ = torque_arb_r;
        
        return F_arb;
    }
    
    Eigen::VectorXd compute_derivatives(const State8DOF_Full& state, const DriverInput& input) {
        using namespace physics::tire;
        using namespace physics::suspension;
        
        Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(State8DOF_Full::SIZE);
        
        const double g = 9.81;
        const double m = params_.mass.m_sprung;
        const double Ixx = params_.mass.Ixx;
        const double Iyy = params_.mass.Iyy;
        const double Izz = params_.mass.Izz;
        const double k_tire = params_.tire.k_tire;
        const double c_tire = params_.tire.c_tire;
        
        const double a = params_.mass.a;
        const double b = params_.mass.b;
        const double tf = params_.mass.track_f / 2;
        const double tr = params_.mass.track_r / 2;
        
        const std::array<Eigen::Vector3d, 4> wheel_pos = {
            Eigen::Vector3d(a, -tf, 0),
            Eigen::Vector3d(a, tf, 0),
            Eigen::Vector3d(-b, -tr, 0),
            Eigen::Vector3d(-b, tr, 0)
        };
        
        const auto [R, E] = compute_rotation_matrices(state.phi, state.theta, state.psi);
        
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
        
        const double delta = input.steering;
        const std::array<double, 4> steer = {delta, delta, 0, 0};
        
        std::array<double, 4> Fx_wheel, Fy_wheel, F_spring, F_damper_total, Fz_tire, susp_travel;
        std::array<double, 4> dz_damper, dz_friction;
        
        for (int i = 0; i < 4; ++i) {
            // サスペンション変位・速度
            double z_sprung_at_wheel = state.z - wheel_pos[i].x() * std::sin(state.theta) 
                                      + wheel_pos[i].y() * std::sin(state.phi);
            susp_travel[i] = z_sprung_at_wheel - state.z_u[i];
            
            double vz_sprung_at_wheel = state.vz 
                - wheel_pos[i].x() * state.q * std::cos(state.theta)
                + wheel_pos[i].y() * state.p * std::cos(state.phi);
            double susp_velocity = vz_sprung_at_wheel - state.vz_u[i];
            
            // スプリング力
            double mr = params_.suspension.motion_ratio[i];
            double k_eff = params_.suspension.k_spring[i] * mr * mr;
            double susp_travel_from_eq = susp_travel[i] - (z_equilibrium_ - z_u_equilibrium_[i]);
            F_spring[i] = static_Fz_[i] + k_eff * susp_travel_from_eq;
            
            // Bouc-Wenダンパー力
            auto bw_out = compute_bouc_wen(
                bw_params_[i],
                susp_travel_from_eq,
                susp_velocity,
                state.damper_z[i]
            );
            dz_damper[i] = bw_out.dz_dt;
            bw_forces_[i] = bw_out.force;
            
            // LuGre摩擦力
            auto lugre_out = compute_lugre(
                lugre_params_[i],
                susp_velocity,
                state.friction_z[i]
            );
            dz_friction[i] = lugre_out.dz_dt;
            lugre_forces_[i] = lugre_out.force;
            
            // 合計ダンパー力 = Bouc-Wen + LuGre
            F_damper_total[i] = bw_out.force + lugre_out.force;
            
            // タイヤ垂直力
            double tire_deflection = state.z_u[i] - z_u_equilibrium_[i];
            double tire_velocity = state.vz_u[i];
            Fz_tire[i] = static_Fz_[i] + params_.mass.m_unsprung[i] * g
                       + k_tire * tire_deflection + c_tire * tire_velocity;
            Fz_tire[i] = std::max(Fz_tire[i], 0.0);
            
            // タイヤ力
            double vx_contact = state.vx - state.r * wheel_pos[i].y();
            double vy_contact = state.vy + state.r * wheel_pos[i].x();
            
            double c = std::cos(steer[i]);
            double s = std::sin(steer[i]);
            double vx_wheel = c * vx_contact + s * vy_contact;
            double vy_wheel = -s * vx_contact + c * vy_contact;
            
            double omega_r = state.omega[i] * params_.tire.radius;
            double vsx = vx_wheel - omega_r;
            double vsy = vy_wheel;
            double vcx = std::abs(omega_r);
            
            TMeasyState<double> tire_state;
            tire_state.xi = state.tire_xi[i];
            tire_state.eta = state.tire_eta[i];
            
            auto tire_out = compute_tmeasy(tire_params_, tire_state, Fz_tire[i],
                                          vsx, vsy, std::max(vcx, 0.1));
            
            Fx_wheel[i] = c * tire_out.Fx - s * tire_out.Fy;
            Fy_wheel[i] = s * tire_out.Fx + c * tire_out.Fy;
            
            dxdt(24 + i) = tire_out.dxi_dt;
            dxdt(28 + i) = tire_out.deta_dt;
            dxdt(32 + i) = dz_damper[i];
            dxdt(36 + i) = dz_friction[i];
            
            // ばね下運動 (SAE Z-down: 重力+ばね力-タイヤ反力)
            double m_u = params_.mass.m_unsprung[i];
            double F_susp_on_unsprung = F_spring[i] + F_damper_total[i];
            double az_u = g + (F_susp_on_unsprung - Fz_tire[i]) / m_u;
            
            dxdt(12 + i) = state.vz_u[i];
            dxdt(16 + i) = az_u;
        }
        
        // ARB
        auto F_arb = compute_arb_forces(susp_travel);
        
        // 合計力とモーメント
        double Fx_total = 0, Fy_total = 0, Fz_total = 0;
        double Mx_total = 0, My_total = 0, Mz_total = 0;
        
        const double h_cg = params_.mass.h_cg;
        
        for (int i = 0; i < 4; ++i) {
            Fx_total += Fx_wheel[i];
            Fy_total += Fy_wheel[i];
            
            double F_susp = F_spring[i] + F_damper_total[i] + F_arb[i];
            Fz_total -= F_susp;
            
            Mx_total -= wheel_pos[i].y() * F_susp;
            Mx_total += Fy_wheel[i] * h_cg;
            
            My_total += wheel_pos[i].x() * F_susp;
            Mz_total += wheel_pos[i].x() * Fy_wheel[i] - wheel_pos[i].y() * Fx_wheel[i];
        }
        
        // 空力
        double v2 = state.vx * state.vx + state.vy * state.vy;
        double Fz_aero = 0.5 * params_.aero.rho * v2 * params_.aero.CL_A;
        double Fx_aero = -0.5 * params_.aero.rho * v2 * params_.aero.CD_A;
        
        Fx_total += Fx_aero;
        Fz_total += Fz_aero;
        
        // 重力
        Eigen::Vector3d g_inertial(0, 0, g);
        Eigen::Vector3d g_body = R * g_inertial;
        
        // ばね上運動
        double ax = Fx_total / m + g_body.x() + state.r * state.vy - state.q * state.vz;
        double ay = Fy_total / m + g_body.y() - state.r * state.vx + state.p * state.vz;
        double az = Fz_total / m + g_body.z() + state.q * state.vx - state.p * state.vy;
        
        dxdt(6) = ax;
        dxdt(7) = ay;
        dxdt(8) = az;
        
        dxdt(9) = Mx_total / Ixx;
        dxdt(10) = My_total / Iyy;
        dxdt(11) = Mz_total / Izz;
        
        // 車輪回転
        double Iw = params_.mass.Iw;
        double R_tire = params_.tire.radius;
        
        double engine_torque = input.throttle * params_.powertrain.max_torque;
        double drive_torque = engine_torque * params_.powertrain.gear_ratio 
                            * params_.powertrain.efficiency / 2;
        
        for (int i = 0; i < 4; ++i) {
            double T_drive = (i >= 2) ? drive_torque : 0;
            
            double c = std::cos(steer[i]);
            double s = std::sin(steer[i]);
            double Fx_w = c * Fx_wheel[i] + s * Fy_wheel[i];
            double T_tire = -Fx_w * R_tire;
            
            dxdt(20 + i) = (T_drive + T_tire) / Iw;
        }
        
        return dxdt;
    }
    
    std::pair<Eigen::Matrix3d, Eigen::Matrix3d> compute_rotation_matrices(
        double phi, double theta, double psi) const {
        
        double cphi = std::cos(phi), sphi = std::sin(phi);
        double cth = std::cos(theta), sth = std::sin(theta);
        double cpsi = std::cos(psi), spsi = std::sin(psi);
        
        Eigen::Matrix3d R;
        R << cpsi*cth, spsi*cth, -sth,
             cpsi*sth*sphi - spsi*cphi, spsi*sth*sphi + cpsi*cphi, cth*sphi,
             cpsi*sth*cphi + spsi*sphi, spsi*sth*cphi - cpsi*sphi, cth*cphi;
        
        double tth = std::tan(theta);
        Eigen::Matrix3d E;
        E << 1, sphi*tth, cphi*tth,
             0, cphi, -sphi,
             0, sphi/cth, cphi/cth;
        
        return {R, E};
    }
};

} // namespace fsae::vehicle
