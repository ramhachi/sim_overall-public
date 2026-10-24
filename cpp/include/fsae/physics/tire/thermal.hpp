#pragma once
/// @file thermal.hpp
/// @brief タイヤ熱モデル（温度依存摩擦係数）

#include <cmath>
#include <algorithm>

namespace fsae::physics::tire {

/// 熱モデルパラメータ
template<typename Scalar = double>
struct TireThermalParams {
    // 熱物性
    Scalar mass_tread = 1.5;        // トレッド質量 [kg]
    Scalar Cp = 1500.0;             // 比熱 [J/(kg·K)]
    Scalar contact_area = 0.015;    // 接地面積 [m²]
    
    // 摩擦係数の温度依存性
    Scalar T_opt = 80.0;            // 最適温度 [°C]
    Scalar T_width = 25.0;          // 温度幅 [°C]
    Scalar mu_max = 1.8;            // 最大摩擦係数
    Scalar mu_min = 1.2;            // 最小摩擦係数（過熱/低温時）
    
    // 熱伝達
    Scalar h_conv = 50.0;           // 対流熱伝達係数 [W/(m²·K)]
    Scalar h_cond = 200.0;          // 伝導熱伝達係数 [W/(m²·K)]
    Scalar T_ambient = 25.0;        // 環境温度 [°C]
    Scalar T_road = 30.0;           // 路面温度 [°C]
};

/// タイヤ温度状態
template<typename Scalar = double>
struct TireThermalState {
    Scalar T_surface = 25.0;  // 表面温度 [°C]
};

/// 熱モデル出力
template<typename Scalar = double>
struct TireThermalOutput {
    Scalar dT_dt;       // 温度変化率 [°C/s]
    Scalar mu_factor;   // 摩擦係数スケール [0-1]
};

/// 摩擦仕事（発熱源）を計算
template<typename Scalar>
Scalar compute_friction_power(Scalar Fx, Scalar Fy, Scalar vsx, Scalar vsy) {
    return std::abs(Fx * vsx) + std::abs(Fy * vsy);
}

/// 温度依存の摩擦係数スケールを計算
/// @return 0〜1のスケールファクター（1が最大グリップ）
template<typename Scalar>
Scalar compute_mu_temperature_factor(
    const TireThermalParams<Scalar>& params,
    Scalar T_surface)
{
    using std::exp;
    Scalar dT = T_surface - params.T_opt;
    Scalar factor = exp(-(dT * dT) / (Scalar(2.0) * params.T_width * params.T_width));
    return params.mu_min / params.mu_max + 
           (Scalar(1.0) - params.mu_min / params.mu_max) * factor;
}

/// タイヤ温度の時間発展を計算
template<typename Scalar>
TireThermalOutput<Scalar> compute_tire_thermal(
    const TireThermalParams<Scalar>& params,
    const TireThermalState<Scalar>& state,
    Scalar Fx, Scalar Fy,
    Scalar vsx, Scalar vsy,
    Scalar vehicle_speed)
{
    using std::abs, std::max, std::pow;
    
    // 発熱（摩擦仕事）
    Scalar Q_gen = compute_friction_power(Fx, Fy, vsx, vsy);
    
    // 対流冷却（速度依存）
    Scalar h_eff = params.h_conv * pow(max(vehicle_speed, Scalar(1.0)), Scalar(0.8));
    Scalar Q_conv = h_eff * params.contact_area * (state.T_surface - params.T_ambient);
    
    // 伝導冷却（路面へ）
    Scalar Q_cond = params.h_cond * params.contact_area * (state.T_surface - params.T_road);
    
    // 熱収支
    Scalar dT_dt = (Q_gen - Q_conv - Q_cond) / (params.mass_tread * params.Cp);
    
    // 摩擦係数ファクター
    Scalar mu_factor = compute_mu_temperature_factor(params, state.T_surface);
    
    return {dT_dt, mu_factor};
}

} // namespace fsae::physics::tire
