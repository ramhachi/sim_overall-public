#pragma once
/// @file steering_torque.hpp
/// @brief ステアリング操舵トルクモデル（Phase A2）
///
/// 物理モデル:
///   I_steer · d²δ_sw/dt² = T_driver + T_SAT - T_friction - T_damping - T_return
///
/// where:
///   T_SAT       = Σ(Mz_tire_i) / R_steer  (符号付きタイヤMz。左旋回で正)
///   T_friction  = T_fric · tanh((dδ_sw/dt)/ε)  (平滑化クーロン摩擦)
///   T_damping   = C_steer · dδ_sw/dt       (粘性減衰)
///   T_return    = K_return · δ_sw          (復元ばね、通常0)

#include <cmath>
#include <algorithm>

namespace fsae::vehicle {

/// @brief ステアリングホイール加速度を計算（トルク平衡から）
/// @param I_steer ステアリング慣性 [kg·m²]
/// @param T_driver ドライバー入力トルク [Nm]
/// @param T_SAT_total 全タイヤのセルフアライニングトルク合計 [Nm]（ホイール座標系）
/// @param R_steer 総ステアリング比 [-]
/// @param delta_sw 現在のステアリングホイール角 [rad]
/// @param ddelta_sw 現在のステアリングホイール角速度 [rad/s]
/// @param T_fric クーロン摩擦トルク [Nm]
/// @param C_steer 粘性減衰係数 [Nm·s/rad]
/// @param K_return 復元ばね剛性 [Nm/rad]（デフォルト: 0）
/// @return ステアリングホイール角加速度 [rad/s²]
inline double compute_steering_acceleration(
    double I_steer,
    double T_driver,
    double T_SAT_total,
    double R_steer,
    double delta_sw,
    double ddelta_sw,
    double T_fric,
    double C_steer,
    double K_return = 0.0)
{
    // SATをステアリングホイールに反映（ギア比で割る）
    double T_SAT_reflected = T_SAT_total / R_steer;

    // 粘性減衰
    double T_damping = C_steer * ddelta_sw;

    // 復元トルク（通常0、セルフセンタリングはSATで実現）
    double T_return_torque = K_return * delta_sw;

    // クーロン摩擦（速度の符号に応じて決定。チャタリング防止のためtanhで平滑化）
    constexpr double epsilon_ddelta = 0.05; // 平滑化閾値 [rad/s]
    double T_friction = T_fric * std::tanh(ddelta_sw / epsilon_ddelta);

    // 運動方程式: I·α = T_driver + T_SAT - T_friction - T_damping - T_return
    // T_SAT_total is the signed tire Mz sum: positive for a left turn and
    // negative for a right turn, so adding it restores the steering angle.
    double ddelta_sw_accel = (T_driver + T_SAT_reflected - T_friction
                             - T_damping - T_return_torque) / I_steer;

    return ddelta_sw_accel;
}

/// @brief ステアリングホイール角からタイヤ舵角への変換
/// @param delta_sw ステアリングホイール角 [rad]
/// @param R_steer 総ステアリング比 [-]
/// @return タイヤ舵角 [rad]
inline double steering_wheel_to_tire_angle(double delta_sw, double R_steer) {
    return delta_sw / R_steer;
}

/// @brief タイヤ舵角からステアリングホイール角への変換
/// @param delta_tire タイヤ舵角 [rad]
/// @param R_steer 総ステアリング比 [-]
/// @return ステアリングホイール角 [rad]
inline double tire_angle_to_steering_wheel(double delta_tire, double R_steer) {
    return delta_tire * R_steer;
}

} // namespace fsae::vehicle
