// src/math/rotation.cpp
#include "fsae/math/rotation.hpp"

namespace fsae::math {

// テンプレートの明示的インスタンス化
template Eigen::Matrix<double, 3, 3> euler_to_rotation_matrix(const EulerAngles<double>&);
template Eigen::Matrix<float, 3, 3> euler_to_rotation_matrix(const EulerAngles<float>&);

template Eigen::Matrix<double, 3, 3> angular_velocity_to_euler_rate(const EulerAngles<double>&);
template Eigen::Matrix<float, 3, 3> angular_velocity_to_euler_rate(const EulerAngles<float>&);

template Eigen::Matrix<double, 3, 1> gravity_in_body(const EulerAngles<double>&, double);
template Eigen::Matrix<float, 3, 1> gravity_in_body(const EulerAngles<float>&, float);

} // namespace fsae::math
