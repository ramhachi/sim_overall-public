#pragma once
/// @file bouc_wen.hpp
/// @brief Bouc-Wenダンパーヒステリシスモデル

#include <cmath>

namespace fsae::physics::suspension {

/// Bouc-Wenパラメータ
template<typename Scalar = double>
struct BoucWenParams {
    // 基本減衰特性
    Scalar c0 = 2000.0;     // 粘性減衰係数 [Ns/m]
    Scalar k0 = 1000.0;     // ばね剛性 [N/m] (通常0)
    
    // ヒステリシスパラメータ
    Scalar alpha = 5000.0;  // ヒステリシス強度
    Scalar A = 1.0;         // 形状パラメータ
    Scalar beta = 500.0;    // 形状パラメータ
    Scalar gamma = 100.0;   // 形状パラメータ
    Scalar n = 2.0;         // 形状指数
    
    // 参照点
    Scalar x0 = 0.0;        // 中立位置
};

/// Bouc-Wen減衰力を計算
/// @param params モデルパラメータ
/// @param x 変位 [m]
/// @param v 速度 [m/s]
/// @param z 履歴変数（状態）
/// @return 減衰力 [N]
template<typename Scalar>
Scalar compute_bouc_wen_force(
    const BoucWenParams<Scalar>& params,
    Scalar x, Scalar v, Scalar z)
{
    return params.c0 * v + params.k0 * (x - params.x0) + params.alpha * z;
}

/// 履歴変数の時間微分を計算
/// @param params モデルパラメータ
/// @param v 速度 [m/s]
/// @param z 履歴変数
/// @return dz/dt
template<typename Scalar>
Scalar compute_bouc_wen_dz(
    const BoucWenParams<Scalar>& params,
    Scalar v, Scalar z)
{
    using std::abs, std::pow;
    
    Scalar abs_z_n_minus_1 = pow(abs(z), params.n - Scalar(1.0));
    Scalar abs_z_n = abs_z_n_minus_1 * abs(z);
    
    return params.A * v 
           - params.beta * abs(v) * z * abs_z_n_minus_1
           - params.gamma * v * abs_z_n;
}

/// Bouc-Wen完全計算（力と微分）
template<typename Scalar>
struct BoucWenOutput {
    Scalar force;   // 減衰力 [N]
    Scalar dz_dt;   // 履歴変数微分
};

template<typename Scalar>
BoucWenOutput<Scalar> compute_bouc_wen(
    const BoucWenParams<Scalar>& params,
    Scalar x, Scalar v, Scalar z)
{
    return {
        compute_bouc_wen_force(params, x, v, z),
        compute_bouc_wen_dz(params, v, z)
    };
}

} // namespace fsae::physics::suspension
