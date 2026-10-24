#pragma once
/// @file vec3.hpp
/// @brief 3次元ベクトル（SIMD最適化対応）

#include <cmath>
#include <array>

namespace fsae::math {

template<typename Scalar = double>
struct Vec3 {
    alignas(32) Scalar x, y, z;
    
    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(Scalar x_, Scalar y_, Scalar z_) : x(x_), y(y_), z(z_) {}
    
    // 配列アクセス
    Scalar& operator[](int i) { return (&x)[i]; }
    const Scalar& operator[](int i) const { return (&x)[i]; }
    
    // 算術演算
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(Scalar s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(Scalar s) const { return {x / s, y / s, z / s}; }
    
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(Scalar s) { x *= s; y *= s; z *= s; return *this; }
    
    // ベクトル演算
    Scalar dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    
    Scalar norm() const { return std::sqrt(x*x + y*y + z*z); }
    Scalar norm_sq() const { return x*x + y*y + z*z; }
    
    Vec3 normalized() const {
        Scalar n = norm();
        return (n > Scalar(1e-12)) ? (*this / n) : Vec3{};
    }
    
    // ゼロベクトル
    static constexpr Vec3 zero() { return {0, 0, 0}; }
    static constexpr Vec3 unit_x() { return {1, 0, 0}; }
    static constexpr Vec3 unit_y() { return {0, 1, 0}; }
    static constexpr Vec3 unit_z() { return {0, 0, 1}; }
};

// スカラー左乗算
template<typename Scalar>
Vec3<Scalar> operator*(Scalar s, const Vec3<Scalar>& v) { return v * s; }

using Vec3d = Vec3<double>;
using Vec3f = Vec3<float>;

} // namespace fsae::math
