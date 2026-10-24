#pragma once
/// @file tmeasy.hpp
/// @brief TMeasyタイヤモデル（過渡特性・低速特異点対策済み）

#include <cmath>
#include <algorithm>

namespace fsae::physics::tire {

/// TMeasyパラメータ
template<typename Scalar = double>
struct TMeasyParams {
    // 基本パラメータ
    Scalar mu_x_max = 1.6;      // 縦方向最大摩擦係数
    Scalar mu_y_max = 2.0;      // 横方向最大摩擦係数（レーシングタイヤ、実測ベース）
    Scalar Fz_nom = 1500.0;     // 基準荷重 [N]
    
    // 緩和長
    Scalar Lx = 0.15;           // 縦緩和長 [m]
    Scalar Ly = 0.25;           // 横緩和長 [m]
    
    // 接地面（カーカス）剛性 [N/m]。定常では xi = Lx·κ, eta = Ly·tanα、かつ
    // 小スリップで mu_eff = s(2−s) ≈ 2s なので、実効スリップ剛性は
    //   Cα = 2·Cy·Ly [N/rad],  Cκ = 2·Cx·Lx [N]。
    Scalar Cx = 80000.0;        // 縦剛性 [N/m] → Cκ = 2·80000·0.15 = 24000 N
    Scalar Cy = 44900.0;        // 横剛性 [N/m] = Cα/(2·Ly)、実測 Cα = 22436 N/rad, Ly = 0.25 m
    
    // 特異点回避用仮想速度
    Scalar v_N = 0.5;           // [m/s]
    
    // 荷重感度
    Scalar dmu_dFz = -0.0001;   // 基準荷重からの荷重増加による摩擦係数低下 [1/N]

    // セルフアライニングトルク近似
    Scalar pneumatic_trail = 0.03; // [m]
};

/// TMeasy状態変数（接地面変形量）
template<typename Scalar = double>
struct TMeasyState {
    Scalar xi = 0;    // 縦変形 [m]
    Scalar eta = 0;   // 横変形 [m]
};

/// TMeasy出力
template<typename Scalar = double>
struct TMeasyOutput {
    Scalar Fx;          // 縦力 [N]
    Scalar Fy;          // 横力 [N]
    Scalar Mz;          // セルフアライニングトルク [Nm]
    Scalar dxi_dt;      // 縦変形速度
    Scalar deta_dt;     // 横変形速度
};

/// TMeasy力計算（純粋関数・テンプレート化で自動微分対応）
/// @param params モデルパラメータ
/// @param state 状態変数（変形量）
/// @param Fz 垂直荷重 [N]
/// @param vsx 縦スリップ速度 [m/s]
/// @param vsy 横スリップ速度 [m/s]
/// @param vcx タイヤ転がり速度 [m/s]
/// @return 力と状態微分
template<typename Scalar>
TMeasyOutput<Scalar> compute_tmeasy(
    const TMeasyParams<Scalar>& params,
    const TMeasyState<Scalar>& state,
    Scalar Fz,
    Scalar vsx,
    Scalar vsy,
    Scalar vcx)
{
    using std::sqrt, std::abs, std::max;
    
    // 荷重クランプ（負荷重対策）
    Fz = max(Fz, Scalar(1.0));
    
    // 変形量の時間微分（一次遅れ系）
    Scalar dxi_dt  = vsx - (abs(vcx) / params.Lx) * state.xi;
    Scalar deta_dt = vsy - (abs(vcx) / params.Ly) * state.eta;
    
    // 荷重依存の摩擦係数
    Scalar delta_Fz = Fz - params.Fz_nom;
    Scalar mu_x = params.mu_x_max + params.dmu_dFz * delta_Fz;
    Scalar mu_y = params.mu_y_max + params.dmu_dFz * delta_Fz;
    mu_x = max(mu_x, Scalar(0.1));
    mu_y = max(mu_y, Scalar(0.1));
    
    // タイヤ変形量の飽和（数値発散防止）
    // vcx→0 のとき減衰項が消失し xi/eta が線形成長 → xi_norm→∞ → NaN
    constexpr Scalar XI_MAX = Scalar(1.0);  // [m] 物理的妥当な上限
    const Scalar xi_safe  = std::clamp(state.xi,  -XI_MAX, XI_MAX);
    const Scalar eta_safe = std::clamp(state.eta, -XI_MAX, XI_MAX);

    // 正規化変形量
    Scalar xi_norm  = params.Cx * xi_safe / (mu_x * Fz + Scalar(1.0));
    Scalar eta_norm = params.Cy * eta_safe / (mu_y * Fz + Scalar(1.0));

    // 合成スリップ（ゼロ除算防止）
    Scalar s = sqrt(xi_norm * xi_norm + eta_norm * eta_norm);
    
    // 摩擦係数（飽和特性）
    Scalar mu_eff;
    if (s < Scalar(1.0)) {
        mu_eff = s * (Scalar(2.0) - s);  // 放物線近似
    } else {
        mu_eff = Scalar(1.0);  // 飽和
    }
    
    // 力計算
    Scalar Fx = Scalar(0), Fy = Scalar(0);
    if (s > Scalar(1e-6)) {
        Fx = -mu_eff * mu_x * Fz * xi_norm / s;
        Fy = -mu_eff * mu_y * Fz * eta_norm / s;
    }
    
    // セルフアライニングトルク（簡易モデル）
    Scalar Mz = -params.pneumatic_trail * Fy;
    
    return {Fx, Fy, Mz, dxi_dt, deta_dt};
}

/// 4輪同時計算（SoA対応バッチ処理）
template<typename Scalar>
void compute_tmeasy_batch(
    const TMeasyParams<Scalar>& params,
    const Scalar* xi,      // [4]
    const Scalar* eta,     // [4]
    const Scalar* Fz,      // [4]
    const Scalar* vsx,     // [4]
    const Scalar* vsy,     // [4]
    const Scalar* vcx,     // [4]
    Scalar* Fx_out,        // [4]
    Scalar* Fy_out,        // [4]
    Scalar* dxi_out,       // [4]
    Scalar* deta_out)      // [4]
{
    for (int i = 0; i < 4; ++i) {
        TMeasyState<Scalar> state{xi[i], eta[i]};
        auto out = compute_tmeasy(params, state, Fz[i], vsx[i], vsy[i], vcx[i]);
        Fx_out[i] = out.Fx;
        Fy_out[i] = out.Fy;
        dxi_out[i] = out.dxi_dt;
        deta_out[i] = out.deta_dt;
    }
}

} // namespace fsae::physics::tire
