#pragma once
/// @file rosenbrock.hpp
/// @brief Rosenbrock法ソルバー（Stiff ODE対応）

#include <Eigen/Dense>
#include <Eigen/LU>
#include <functional>
#include <cmath>

namespace fsae::solver {

/// Rosenbrock23法の係数
struct Rosenbrock23Coeffs {
    static constexpr double gamma = 0.7886751345948129;
    static constexpr double a21 = 1.0;
    static constexpr double c21 = -2.0;
    static constexpr double b1 = 0.5;
    static constexpr double b2 = 0.5;
    static constexpr double e1 = 1.0 / 6.0;
    static constexpr double e2 = -1.0 / 6.0;
};

/// ソルバー設定
struct RosenbrockConfig {
    double dt_init = 1e-3;      // 初期タイムステップ
    double dt_min = 1e-8;       // 最小タイムステップ
    double dt_max = 1e-2;       // 最大タイムステップ
    double atol = 1e-6;         // 絶対許容誤差
    double rtol = 1e-4;         // 相対許容誤差
    int max_iterations = 10000;
};

/// ソルバー結果
struct SolverResult {
    bool success;
    int steps;
    double final_time;
    int jacobian_evals;
};

/// Rosenbrock23ソルバークラス
template<int N>
class Rosenbrock23 {
public:
    using StateVector = Eigen::Matrix<double, N, 1>;
    using JacobianMatrix = Eigen::Matrix<double, N, N>;
    using DerivFunc = std::function<StateVector(double, const StateVector&)>;
    using JacobianFunc = std::function<JacobianMatrix(double, const StateVector&)>;
    
    Rosenbrock23(DerivFunc f, JacobianFunc J, RosenbrockConfig config = {})
        : f_(std::move(f)), J_(std::move(J)), config_(config) {}
    
    /// 1ステップ積分
    /// @param t 現在時刻
    /// @param x 現在状態
    /// @param dt タイムステップ
    /// @return 次の状態
    StateVector step(double t, const StateVector& x, double dt) {
        using C = Rosenbrock23Coeffs;
        
        // ヤコビアン計算
        JacobianMatrix Jac = J_(t, x);
        jacobian_count_++;
        
        // (I - gamma * dt * J)^(-1) を計算
        JacobianMatrix W = JacobianMatrix::Identity() - C::gamma * dt * Jac;
        auto W_lu = W.partialPivLu();
        
        // Stage 1
        StateVector f1 = f_(t, x);
        StateVector k1 = W_lu.solve(dt * f1);
        
        // Stage 2
        StateVector x2 = x + C::a21 * k1;
        StateVector f2 = f_(t + dt, x2);
        StateVector k2 = W_lu.solve(dt * f2 + C::c21 * k1);
        
        // 解の更新
        StateVector x_new = x + C::b1 * k1 + C::b2 * k2;
        
        // 誤差推定（適応ステップ用）
        error_estimate_ = (C::e1 * k1 + C::e2 * k2).norm();
        
        return x_new;
    }
    
    /// 時間区間を積分
    SolverResult integrate(double t0, double t_end, StateVector& x) {
        double t = t0;
        double dt = config_.dt_init;
        int steps = 0;
        jacobian_count_ = 0;
        
        while (t < t_end && steps < config_.max_iterations) {
            // 残り時間でクランプ
            dt = std::min(dt, t_end - t);
            
            // 1ステップ実行
            StateVector x_new = step(t, x, dt);
            
            // 誤差に基づくステップサイズ調整
            double tol = config_.atol + config_.rtol * x.norm();
            double err_ratio = error_estimate_ / tol;
            
            if (err_ratio < 1.0 || dt <= config_.dt_min) {
                // ステップ受け入れ
                x = x_new;
                t += dt;
                steps++;
                
                // 次のステップサイズ
                double factor = 0.9 * std::pow(err_ratio, -0.5);
                factor = std::clamp(factor, 0.1, 5.0);
                dt *= factor;
            } else {
                // ステップ拒否、縮小
                dt *= 0.5;
            }
            
            dt = std::clamp(dt, config_.dt_min, config_.dt_max);
        }
        
        return {t >= t_end, steps, t, jacobian_count_};
    }
    
    int jacobian_count() const { return jacobian_count_; }
    
private:
    DerivFunc f_;
    JacobianFunc J_;
    RosenbrockConfig config_;
    double error_estimate_ = 0;
    int jacobian_count_ = 0;
};

/// 動的サイズ版
class RosenbrockDynamic {
public:
    using StateVector = Eigen::VectorXd;
    using JacobianMatrix = Eigen::MatrixXd;
    using DerivFunc = std::function<StateVector(double, const StateVector&)>;
    using JacobianFunc = std::function<JacobianMatrix(double, const StateVector&)>;
    
    RosenbrockDynamic(DerivFunc f, JacobianFunc J, RosenbrockConfig config = {})
        : f_(std::move(f)), J_(std::move(J)), config_(config) {}
    
    /// 1ステップ積分
    StateVector step(double t, const StateVector& x, double dt) {
        using C = Rosenbrock23Coeffs;
        
        const int n = x.size();
        
        // ヤコビアン計算
        JacobianMatrix Jac = J_(t, x);
        jacobian_count_++;
        
        // (I - gamma * dt * J)^(-1) を計算
        JacobianMatrix W = JacobianMatrix::Identity(n, n) - C::gamma * dt * Jac;
        auto W_lu = W.partialPivLu();
        
        // Stage 1
        StateVector f1 = f_(t, x);
        StateVector k1 = W_lu.solve(dt * f1);
        
        // Stage 2
        StateVector x2 = x + C::a21 * k1;
        StateVector f2 = f_(t + dt, x2);
        StateVector k2 = W_lu.solve(dt * f2 + C::c21 * k1);
        
        // 解の更新
        StateVector x_new = x + C::b1 * k1 + C::b2 * k2;
        
        // 誤差推定
        error_estimate_ = (C::e1 * k1 + C::e2 * k2).norm();
        
        return x_new;
    }
    
    /// 時間区間を積分
    SolverResult integrate(double t0, double t_end, StateVector& x) {
        double t = t0;
        double dt = config_.dt_init;
        int steps = 0;
        jacobian_count_ = 0;
        
        while (t < t_end && steps < config_.max_iterations) {
            // 残り時間でクランプ
            dt = std::min(dt, t_end - t);
            
            // 1ステップ実行
            StateVector x_new = step(t, x, dt);
            
            // 誤差に基づくステップサイズ調整
            double tol = config_.atol + config_.rtol * x.norm();
            double err_ratio = error_estimate_ / tol;
            
            if (err_ratio < 1.0 || dt <= config_.dt_min) {
                // ステップ受け入れ
                x = x_new;
                t += dt;
                steps++;
                
                // 次のステップサイズ
                double factor = 0.9 * std::pow(err_ratio, -0.5);
                factor = std::clamp(factor, 0.1, 5.0);
                dt *= factor;
            } else {
                // ステップ拒否、縮小
                dt *= 0.5;
            }
            
            dt = std::clamp(dt, config_.dt_min, config_.dt_max);
        }
        
        return {t >= t_end, steps, t, jacobian_count_};
    }
    
    int jacobian_count() const { return jacobian_count_; }
    
private:
    DerivFunc f_;
    JacobianFunc J_;
    RosenbrockConfig config_;
    double error_estimate_ = 0;
    int jacobian_count_ = 0;
};

} // namespace fsae::solver
