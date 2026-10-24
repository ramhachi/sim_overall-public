#pragma once
/// @file lugre.hpp
/// @brief LuGre摩擦モデル（スティック・スリップ対応）

#include <cmath>

namespace fsae::physics::suspension {

/// LuGreパラメータ
template<typename Scalar = double>
struct LuGreParams {
    Scalar sigma_0 = 100000.0;  // 毛剛性 [N/m]
    Scalar sigma_1 = 400.0;     // 毛減衰 [Ns/m]
    Scalar sigma_2 = 10.0;      // 粘性摩擦 [Ns/m]
    
    Scalar Fc = 50.0;           // クーロン摩擦力 [N]
    Scalar Fs = 60.0;           // 静止摩擦力 [N]
    Scalar vs = 0.01;           // ストライベック速度 [m/s]
};

/// ストライベック関数 g(v)
template<typename Scalar>
Scalar stribeck_function(const LuGreParams<Scalar>& params, Scalar v) {
    using std::abs, std::exp;
    Scalar v_ratio = v / params.vs;
    return params.Fc + (params.Fs - params.Fc) * exp(-(v_ratio * v_ratio));
}

/// LuGre摩擦力を計算
/// @param params モデルパラメータ
/// @param v 速度 [m/s]
/// @param z 毛たわみ状態
/// @return 摩擦力 [N]
template<typename Scalar>
Scalar compute_lugre_force(
    const LuGreParams<Scalar>& params,
    Scalar v, Scalar z, Scalar dz_dt)
{
    return params.sigma_0 * z + params.sigma_1 * dz_dt + params.sigma_2 * v;
}

/// 毛たわみの時間微分を計算
/// @param params モデルパラメータ
/// @param v 速度 [m/s]
/// @param z 毛たわみ状態
/// @return dz/dt
template<typename Scalar>
Scalar compute_lugre_dz(
    const LuGreParams<Scalar>& params,
    Scalar v, Scalar z)
{
    using std::abs;
    Scalar g_v = stribeck_function(params, v);
    Scalar sigma_0_over_g = params.sigma_0 / g_v;
    return v - sigma_0_over_g * abs(v) * z;
}

/// LuGre完全計算（力と微分）
template<typename Scalar>
struct LuGreOutput {
    Scalar force;   // 摩擦力 [N]
    Scalar dz_dt;   // 毛たわみ微分
};

template<typename Scalar>
LuGreOutput<Scalar> compute_lugre(
    const LuGreParams<Scalar>& params,
    Scalar v, Scalar z)
{
    Scalar dz_dt = compute_lugre_dz(params, v, z);
    Scalar force = compute_lugre_force(params, v, z, dz_dt);
    return {force, dz_dt};
}

/// サブステップ積分付きLuGre計算（ガイド §12.6）
/// @brief 高剛性（σ_0 > 2000 N/m）の場合に安定性を確保
/// @param params モデルパラメータ
/// @param v 速度 [m/s]
/// @param z_initial 初期毛たわみ状態
/// @param dt メインタイムステップ [s]
/// @param substeps サブステップ数（1=通常、10=推奨）
/// @return 最終状態での力と微分
template<typename Scalar>
LuGreOutput<Scalar> compute_lugre_substep(
    const LuGreParams<Scalar>& params,
    Scalar v, Scalar z_initial, Scalar dt, int substeps = 10)
{
    using std::abs;

    // サブステップ積分（簡易Euler法）
    Scalar z = z_initial;
    Scalar sub_dt = dt / static_cast<Scalar>(substeps);

    for (int i = 0; i < substeps; ++i) {
        Scalar dz_dt = compute_lugre_dz(params, v, z);
        z += dz_dt * sub_dt;
    }

    // 最終的な力を計算
    Scalar dz_dt_final = compute_lugre_dz(params, v, z);
    Scalar force = compute_lugre_force(params, v, z, dz_dt_final);

    return {force, dz_dt_final};
}

/// 自動選択版：σ_0に応じてサブステップを適応的に選択
/// @brief σ_0 < 2000 なら通常版、超過ならサブステップ版を使用
template<typename Scalar>
LuGreOutput<Scalar> compute_lugre_adaptive(
    const LuGreParams<Scalar>& params,
    Scalar v, Scalar z, Scalar dt = 0.001)
{
    constexpr Scalar STABILITY_THRESHOLD = 2000.0;  // 2/Δt for Δt=1ms

    if (params.sigma_0 < STABILITY_THRESHOLD) {
        // 安定領域：通常計算
        return compute_lugre(params, v, z);
    } else {
        // 不安定領域：サブステップ積分
        int substeps = static_cast<int>(params.sigma_0 / STABILITY_THRESHOLD) + 1;
        substeps = std::min(substeps, 20);  // 最大20ステップに制限
        return compute_lugre_substep(params, v, z, dt, substeps);
    }
}

} // namespace fsae::physics::suspension
