// src/math/spline.cpp
#include "fsae/math/spline.hpp"

namespace fsae::math {

// 明示的インスタンス化
template class CubicSpline<double>;
template class CubicSpline<float>;
template class BicubicSpline<double>;
template class BicubicSpline<float>;

} // namespace fsae::math
