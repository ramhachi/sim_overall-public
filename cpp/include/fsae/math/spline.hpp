#pragma once
/// @file spline.hpp
/// @brief 3次スプライン補間（C^2連続）

#include <vector>
#include <stdexcept>
#include <cmath>

namespace fsae::math {

/// 1次元3次スプライン補間
template<typename Scalar = double>
class CubicSpline {
public:
    CubicSpline() = default;
    
    /// スプライン係数を計算
    /// @param x 独立変数（単調増加）
    /// @param y 従属変数
    void build(const std::vector<Scalar>& x, const std::vector<Scalar>& y) {
        if (x.size() != y.size() || x.size() < 2) {
            throw std::invalid_argument("Invalid input size");
        }
        
        x_ = x;
        y_ = y;
        n_ = x.size();
        
        // トーマスアルゴリズムで2階微分を計算
        std::vector<Scalar> h(n_ - 1);
        for (size_t i = 0; i < n_ - 1; ++i) {
            h[i] = x_[i + 1] - x_[i];
        }
        
        // 三重対角行列の係数
        std::vector<Scalar> alpha(n_ - 1);
        for (size_t i = 1; i < n_ - 1; ++i) {
            alpha[i] = Scalar(3.0) / h[i] * (y_[i + 1] - y_[i]) 
                     - Scalar(3.0) / h[i - 1] * (y_[i] - y_[i - 1]);
        }
        
        // 前進消去
        std::vector<Scalar> l(n_), mu(n_), z(n_);
        l[0] = Scalar(1.0);
        mu[0] = z[0] = Scalar(0.0);
        
        for (size_t i = 1; i < n_ - 1; ++i) {
            l[i] = Scalar(2.0) * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        
        l[n_ - 1] = Scalar(1.0);
        z[n_ - 1] = Scalar(0.0);
        
        // 後退代入
        c_.resize(n_);
        b_.resize(n_ - 1);
        d_.resize(n_ - 1);
        
        c_[n_ - 1] = Scalar(0.0);
        for (int i = n_ - 2; i >= 0; --i) {
            c_[i] = z[i] - mu[i] * c_[i + 1];
            b_[i] = (y_[i + 1] - y_[i]) / h[i] - h[i] * (c_[i + 1] + Scalar(2.0) * c_[i]) / Scalar(3.0);
            d_[i] = (c_[i + 1] - c_[i]) / (Scalar(3.0) * h[i]);
        }
        
        built_ = true;
    }
    
    /// 補間値を計算
    Scalar operator()(Scalar x) const {
        if (!built_) throw std::runtime_error("Spline not built");
        
        // 区間を探索
        size_t i = find_interval(x);
        Scalar dx = x - x_[i];
        
        return y_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }
    
    /// 1階微分を計算
    Scalar derivative(Scalar x) const {
        if (!built_) throw std::runtime_error("Spline not built");
        
        size_t i = find_interval(x);
        Scalar dx = x - x_[i];
        
        return b_[i] + Scalar(2.0) * c_[i] * dx + Scalar(3.0) * d_[i] * dx * dx;
    }
    
private:
    size_t find_interval(Scalar x) const {
        // 範囲外はクランプ
        if (x <= x_[0]) return 0;
        if (x >= x_[n_ - 1]) return n_ - 2;
        
        // 二分探索
        size_t lo = 0, hi = n_ - 1;
        while (hi - lo > 1) {
            size_t mid = (lo + hi) / 2;
            if (x_[mid] > x) hi = mid;
            else lo = mid;
        }
        return lo;
    }
    
    std::vector<Scalar> x_, y_;
    std::vector<Scalar> b_, c_, d_;  // スプライン係数
    size_t n_ = 0;
    bool built_ = false;
};

/// 2次元スプライン補間（バイキュービック）
template<typename Scalar = double>
class BicubicSpline {
public:
    /// グリッドデータからスプライン構築
    void build(
        const std::vector<Scalar>& x,
        const std::vector<Scalar>& y,
        const std::vector<std::vector<Scalar>>& z)
    {
        x_ = x;
        y_ = y;
        nx_ = x.size();
        ny_ = y.size();
        
        // 各行に対してxスプラインを構築
        row_splines_.resize(ny_);
        for (size_t j = 0; j < ny_; ++j) {
            row_splines_[j].build(x_, z[j]);
        }
        
        built_ = true;
    }
    
    /// 補間値を計算
    Scalar operator()(Scalar x, Scalar y) const {
        if (!built_) throw std::runtime_error("Spline not built");
        
        // まず各行でx補間
        std::vector<Scalar> row_values(ny_);
        for (size_t j = 0; j < ny_; ++j) {
            row_values[j] = row_splines_[j](x);
        }
        
        // y方向でスプライン補間
        CubicSpline<Scalar> col_spline;
        col_spline.build(y_, row_values);
        
        return col_spline(y);
    }
    
private:
    std::vector<Scalar> x_, y_;
    std::vector<CubicSpline<Scalar>> row_splines_;
    size_t nx_ = 0, ny_ = 0;
    bool built_ = false;
};

} // namespace fsae::math
