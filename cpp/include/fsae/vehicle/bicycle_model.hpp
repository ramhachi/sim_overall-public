#pragma once
/// @file bicycle_model.hpp
/// @brief 6-DOF動的自転車モデル（MPC予測用）
///
/// Phase 8a: Model Predictive Control実装
/// 14DOFモデルの簡略版：高速かつ十分な精度でMPC予測に使用
///
/// 状態: x = [x, y, ψ, vx, vy, r]  (6自由度)
/// 制御: u = [δ, ax]               (2入力)
///
/// 参考: Rajamani "Vehicle Dynamics and Control" (2012), Chapter 2

#include <Eigen/Dense>
#include <cmath>
#include "vehicle_params.hpp"

namespace fsae::vehicle {

/// 6自由度自転車モデル状態
struct BicycleState6DOF {
    double x;      ///< グローバルX位置 [m]
    double y;      ///< グローバルY位置 [m]
    double psi;    ///< ヨー角 [rad]
    double vx;     ///< 車体X方向速度 [m/s]
    double vy;     ///< 車体Y方向速度 [m/s]
    double r;      ///< ヨーレート [rad/s]

    /// Eigenベクトルに変換
    Eigen::VectorXd to_vector() const {
        Eigen::VectorXd vec(6);
        vec << x, y, psi, vx, vy, r;
        return vec;
    }

    /// Eigenベクトルから復元
    static BicycleState6DOF from_vector(const Eigen::VectorXd& vec) {
        return {vec(0), vec(1), vec(2), vec(3), vec(4), vec(5)};
    }
};

/// 自転車モデル制御入力
struct BicycleControl {
    double delta;  ///< 前輪舵角 [rad] (正=右旋回、車両モデル慣例に合わせる)
    double ax;     ///< 縦加速度 [m/s²] (正=加速、負=減速)

    /// Eigenベクトルに変換
    Eigen::VectorXd to_vector() const {
        Eigen::VectorXd vec(2);
        vec << delta, ax;
        return vec;
    }

    /// Eigenベクトルから復元
    static BicycleControl from_vector(const Eigen::VectorXd& vec) {
        return {vec(0), vec(1)};
    }
};

/// 自転車モデルパラメータ
struct BicycleParams {
    double m;       ///< 車両質量 [kg]
    double Izz;     ///< ヨー慣性モーメント [kg·m²]
    double lf;      ///< 前軸から重心までの距離 [m]
    double lr;      ///< 後軸から重心までの距離 [m]
    double Cf;      ///< 前輪コーナリング剛性 [N/rad]
    double Cr;      ///< 後輪コーナリング剛性 [N/rad]
    double pneumatic_trail = 0.0; ///< タイヤ空気トレール [m]

    /// 14DOFパラメータから自転車パラメータを抽出
    static BicycleParams from_vehicle_params(const VehicleParams& vp) {
        BicycleParams bp;
        bp.m = vp.mass.m_total;
        bp.Izz = vp.mass.Izz;
        bp.lf = vp.mass.a;
        bp.lr = vp.mass.b;

        // Linearize the TMeasy fallback around zero slip. At steady state,
        // eta = Ly * alpha and the parabolic force law has slope
        // dFy/dalpha = 2 * Cy * Ly * mu_y * Fz / (mu_y * Fz + 1)
        // per tire. Preserve the 14DOF load/temperature-dependent Ly contract
        // so the MPC surrogate follows tire parameter changes.
        const auto effective_lateral_relaxation_length = [&vp](double fz) {
            const auto& relax = vp.tire.relaxation;
            const double base = relax.Ly_base;
            if (!relax.enabled) {
                return std::clamp(base, relax.min_Ly, relax.max_Ly);
            }

            const double fz_ratio = std::max(fz, 1.0) / std::max(vp.tire.Fz_nom, 1.0);
            const double load_scale = std::pow(fz_ratio, relax.Fz_exponent);
            const double temp_scale =
                1.0 + relax.temp_coeff_per_c * (vp.tire.ambient_temperature_c - relax.temp_ref_c);
            return std::clamp(base * load_scale * temp_scale, relax.min_Ly, relax.max_Ly);
        };
        const auto linearized_tire_cornering_stiffness =
            [&vp, &effective_lateral_relaxation_length](double fz) {
                const double mu_y =
                    std::max(vp.tire.mu_y_max + vp.tire.dmu_dFz * (fz - vp.tire.Fz_nom), 0.1);
                const double force_scale = mu_y * fz / (mu_y * fz + 1.0);
                return 2.0 * vp.tire.Cy * effective_lateral_relaxation_length(fz) * force_scale;
            };

        const double g = 9.81;
        const double wheelbase = bp.lf + bp.lr;
        const double front_sprung_load = vp.mass.m_sprung * g * bp.lr / wheelbase / 2.0;
        const double rear_sprung_load = vp.mass.m_sprung * g * bp.lf / wheelbase / 2.0;
        bp.Cf = linearized_tire_cornering_stiffness(
                    front_sprung_load + vp.mass.m_unsprung[0] * g)
              + linearized_tire_cornering_stiffness(
                    front_sprung_load + vp.mass.m_unsprung[1] * g);
        bp.Cr = linearized_tire_cornering_stiffness(
                    rear_sprung_load + vp.mass.m_unsprung[2] * g)
              + linearized_tire_cornering_stiffness(
                    rear_sprung_load + vp.mass.m_unsprung[3] * g);
        bp.pneumatic_trail = vp.tire.pneumatic_trail;

        return bp;
    }
};

/// 6自由度動的自転車モデル
///
/// 動力学方程式:
///   ẋ = vx·cos(ψ) - vy·sin(ψ)
///   ẏ = vx·sin(ψ) + vy·cos(ψ)
///   ψ̇ = r
///   v̇x = ax + vy·r                    # コリオリ項
///   v̇y = (Fyf + Fyr)/m - vx·r         # 横力とコリオリ項
///   ṙ = (lf·Fyf - lr·Fyr)/Izz         # ヨーモーメント
///
/// タイヤモデル（線形化）:
///   Fyf = -Cf·αf,  Fyr = -Cr·αr
///   αf = δ - (vy + lf·r)/vx           # 前輪スリップ角
///   αr = -(vy - lr·r)/vx              # 後輪スリップ角
class BicycleModel6DOF {
public:
    explicit BicycleModel6DOF(const BicycleParams& params)
        : params_(params) {}

    /// 連続時間状態微分を計算: ẋ = f(x, u)
    /// @param state 現在状態
    /// @param control 制御入力
    /// @return 状態微分 dx/dt
    BicycleState6DOF compute_derivative(
        const BicycleState6DOF& state,
        const BicycleControl& control) const
    {
        // 低速での数値安定性（vx→0でスリップ角が特異点）
        // 1.0 m/s未満では線形タイヤモデルの仮定が崩れるため、最小速度を設定
        // 自転車モデルは前進走行を仮定 → 常に正にクランプ
        double vx_safe = std::max(state.vx, 1.0);

        // スリップ角計算
        // Phase 7慣例: 正のδ = 右旋回
        // SAE慣例: 正のδ = 左旋回
        // → δの符号を反転してから標準式を適用
        double delta_sae = -control.delta;  // Phase 7 → SAE変換

        // スリップ角（SAE慣例）: α = atan2(vy_tire, vx) - δ
        // 線形近似: α ≈ (vy_tire / vx) - δ
        // 前輪: vy_tire = vy + lf*r
        // 後輪: vy_tire = vy - lr*r
        double alpha_f = (state.vy + params_.lf * state.r) / vx_safe - delta_sae;
        double alpha_r = (state.vy - params_.lr * state.r) / vx_safe;

        // スリップ角飽和（線形タイヤモデルの有効範囲を超えないように）
        // 実タイヤは約10-15度で力が飽和するため、±0.3 rad（≈17度）でクランプ
        constexpr double alpha_max = 0.3;
        alpha_f = std::clamp(alpha_f, -alpha_max, alpha_max);
        alpha_r = std::clamp(alpha_r, -alpha_max, alpha_max);

        // タイヤ横力（復元力: スリップに反対方向）
        double Fyf = -params_.Cf * alpha_f;
        double Fyr = -params_.Cr * alpha_r;

        // 状態微分
        BicycleState6DOF derivative;
        derivative.x = state.vx * std::cos(state.psi) - state.vy * std::sin(state.psi);
        derivative.y = state.vx * std::sin(state.psi) + state.vy * std::cos(state.psi);
        derivative.psi = state.r;
        derivative.vx = control.ax + state.vy * state.r;
        derivative.vy = (Fyf + Fyr) / params_.m - state.vx * state.r;
        const double Mz_axles = -params_.pneumatic_trail * (Fyf + Fyr);
        derivative.r = (params_.lf * Fyf - params_.lr * Fyr + Mz_axles) / params_.Izz;

        return derivative;
    }

    /// RK4積分による1ステップ予測
    /// @param state 現在状態
    /// @param control 制御入力（定値と仮定）
    /// @param dt 時間刻み [s]
    /// @return 次状態 x(t+dt)
    BicycleState6DOF predict_step(
        const BicycleState6DOF& state,
        const BicycleControl& control,
        double dt) const
    {
        // RK4 (4次Runge-Kutta)
        auto add_scaled = [](const BicycleState6DOF& s, const BicycleState6DOF& ds, double scale) {
            return BicycleState6DOF{
                s.x + scale * ds.x,
                s.y + scale * ds.y,
                s.psi + scale * ds.psi,
                s.vx + scale * ds.vx,
                s.vy + scale * ds.vy,
                s.r + scale * ds.r
            };
        };

        auto k1 = compute_derivative(state, control);
        auto k2 = compute_derivative(add_scaled(state, k1, 0.5 * dt), control);
        auto k3 = compute_derivative(add_scaled(state, k2, 0.5 * dt), control);
        auto k4 = compute_derivative(add_scaled(state, k3, dt), control);

        return BicycleState6DOF{
            state.x + (dt / 6.0) * (k1.x + 2*k2.x + 2*k3.x + k4.x),
            state.y + (dt / 6.0) * (k1.y + 2*k2.y + 2*k3.y + k4.y),
            state.psi + (dt / 6.0) * (k1.psi + 2*k2.psi + 2*k3.psi + k4.psi),
            std::max(state.vx + (dt / 6.0) * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx), 0.5),  // vx下限クランプ（前進仮定）
            state.vy + (dt / 6.0) * (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy),
            state.r + (dt / 6.0) * (k1.r + 2*k2.r + 2*k3.r + k4.r)
        };
    }

    /// 線形化: x(t+dt) ≈ A·x(t) + B·u(t)
    /// 有限差分法により離散時間線形化を計算
    /// @param state 線形化点の状態
    /// @param control 線形化点の制御
    /// @param dt 時間刻み [s]
    /// @return (A, B) 行列のペア (6×6, 6×2)
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearize(
        const BicycleState6DOF& state,
        const BicycleControl& control,
        double dt) const
    {
        const int nx = 6;  // 状態次元
        const int nu = 2;  // 制御次元

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);

        // 有限差分の摂動量
        const double eps_x = 1e-5;  // 状態摂動
        const double eps_u = 1e-5;  // 制御摂動

        // 基準軌道
        auto x_nom = predict_step(state, control, dt);
        Eigen::VectorXd x_nom_vec = x_nom.to_vector();

        // A行列: ∂x(t+dt)/∂x(t)
        for (int i = 0; i < nx; ++i) {
            BicycleState6DOF x_pert = state;
            double* x_ptr = reinterpret_cast<double*>(&x_pert);
            x_ptr[i] += eps_x;

            auto x_plus = predict_step(x_pert, control, dt);
            Eigen::VectorXd x_plus_vec = x_plus.to_vector();

            A.col(i) = (x_plus_vec - x_nom_vec) / eps_x;
        }

        // B行列: ∂x(t+dt)/∂u(t)
        for (int i = 0; i < nu; ++i) {
            BicycleControl u_pert = control;
            double* u_ptr = reinterpret_cast<double*>(&u_pert);
            u_ptr[i] += eps_u;

            auto x_plus = predict_step(state, u_pert, dt);
            Eigen::VectorXd x_plus_vec = x_plus.to_vector();

            B.col(i) = (x_plus_vec - x_nom_vec) / eps_u;
        }

        return {A, B};
    }

    /// 定常円旋回の解析解を計算（テスト用）
    /// @param speed 速度 [m/s]
    /// @param radius 旋回半径 [m] (正=左旋回、負=右旋回)
    /// @return (ヨーレート, 横速度, 舵角)
    std::tuple<double, double, double> steady_state_cornering(
        double speed,
        double radius) const
    {
        // 定常円旋回: r = v/R, vy = 0 (理想)
        double yaw_rate = speed / radius;
        double lateral_accel = speed * speed / radius;

        // アンダーステア勾配
        double L = params_.lf + params_.lr;
        double K_us = (params_.m / (2.0 * L)) *
                     (params_.lr / params_.Cf - params_.lf / params_.Cr);

        // 必要舵角（SAE慣例）
        double delta_sae = L / radius + K_us * lateral_accel;

        // Phase 7慣例に変換（正のδ = 右旋回）
        double delta_phase7 = -delta_sae;

        // 実際の横速度（線形モデルでは小さい）
        double vy = 0.0;  // 簡略化（実際は微小な値）

        return {yaw_rate, vy, delta_phase7};
    }

    /// パラメータ取得
    const BicycleParams& params() const { return params_; }

private:
    BicycleParams params_;
};

} // namespace fsae::vehicle
