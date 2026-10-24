#pragma once
/// @file rotation.hpp
/// @brief オイラー角・回転行列（SAE J670e座標系）

#include <cmath>
#include <Eigen/Dense>

namespace fsae::math {

/// オイラー角（Z-Y-X順: Yaw-Pitch-Roll）
template<typename Scalar = double>
struct EulerAngles {
    Scalar phi;    // Roll (X軸周り)
    Scalar theta;  // Pitch (Y軸周り)
    Scalar psi;    // Yaw (Z軸周り)
};

/// 慣性座標系→車両座標系の回転行列を計算
/// @param euler オイラー角（phi, theta, psi）
/// @return 3x3回転行列
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> euler_to_rotation_matrix(const EulerAngles<Scalar>& euler) {
    using std::cos, std::sin;
    
    const Scalar cphi = cos(euler.phi),   sphi = sin(euler.phi);
    const Scalar cth  = cos(euler.theta), sth  = sin(euler.theta);
    const Scalar cpsi = cos(euler.psi),   spsi = sin(euler.psi);
    
    Eigen::Matrix<Scalar, 3, 3> R;
    R << cpsi*cth,                  spsi*cth,                 -sth,
         cpsi*sth*sphi - spsi*cphi, spsi*sth*sphi + cpsi*cphi, cth*sphi,
         cpsi*sth*cphi + spsi*sphi, spsi*sth*cphi - cpsi*sphi, cth*cphi;
    
    return R;
}

/// 角速度→オイラー角速度の変換行列
/// @param euler 現在のオイラー角
/// @return 変換行列 E: [phi_dot, theta_dot, psi_dot]^T = E * [p, q, r]^T
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 3> angular_velocity_to_euler_rate(const EulerAngles<Scalar>& euler) {
    using std::cos, std::sin, std::tan;
    
    const Scalar cphi = cos(euler.phi), sphi = sin(euler.phi);
    const Scalar cth  = cos(euler.theta), tth = tan(euler.theta);
    
    Eigen::Matrix<Scalar, 3, 3> E;
    E << 1, sphi*tth,  cphi*tth,
         0, cphi,     -sphi,
         0, sphi/cth,  cphi/cth;
    
    return E;
}

/// 重力ベクトルを車両座標系で取得（SAE: +Z = 下向き）
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> gravity_in_body(const EulerAngles<Scalar>& euler, Scalar g = 9.81) {
    auto R = euler_to_rotation_matrix(euler);
    Eigen::Matrix<Scalar, 3, 1> g_inertial{0, 0, g};  // SAE: Z下向きが正
    return R * g_inertial;
}

} // namespace fsae::math
