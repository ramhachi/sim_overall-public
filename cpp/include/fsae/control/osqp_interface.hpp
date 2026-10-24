#pragma once
/// @file osqp_interface.hpp
/// @brief OSQP二次計画法ソルバーのC++ラッパー（Phase 8a, OSQP v1.0）
///
/// OSQPを使用してMPC最適化問題を解く:
///   minimize    (1/2)x'Px + q'x
///   subject to  l ≤ Ax ≤ u
///
/// 参考: https://osqp.org/docs/

#ifdef HAS_OSQP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <osqp.h>
#include <memory>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <string>
#include <cstring>

namespace fsae::control {

/// OSQP求解結果
struct OSQPResult {
    Eigen::VectorXd solution;       ///< 最適解 x*
    Eigen::VectorXd dual_var;       ///< 双対変数 y*
    double obj_val;                 ///< 目的関数値
    int status;                     ///< ステータスコード（1=solved）
    int iter;                       ///< 反復回数
    double solve_time_ms;           ///< 求解時間 [ms]
    bool converged;                 ///< 収束判定

    /// ステータスメッセージ
    std::string status_message() const {
        switch (status) {
            case OSQP_SOLVED: return "Solved";
            case OSQP_SOLVED_INACCURATE: return "Solved (inaccurate)";
            case OSQP_MAX_ITER_REACHED: return "Max iterations reached";
            case OSQP_PRIMAL_INFEASIBLE: return "Primal infeasible";
            case OSQP_DUAL_INFEASIBLE: return "Dual infeasible";
            case OSQP_TIME_LIMIT_REACHED: return "Time limit reached";
            default: return "Unknown status";
        }
    }
};

/// OSQP設定パラメータ
struct OSQPConfig {
    double rho = 0.1;               ///< ADMM step size
    double sigma = 1e-6;            ///< ADMM regularization
    int max_iter = 4000;            ///< 最大反復回数
    double eps_abs = 1e-4;          ///< 絶対許容誤差
    double eps_rel = 1e-4;          ///< 相対許容誤差
    double alpha = 1.6;             ///< Relaxation parameter
    bool verbose = false;           ///< 詳細出力
    bool warm_start = true;         ///< ウォームスタート有効化
    bool polish = true;             ///< Solution polishing
    int polish_refine_iter = 3;     ///< Polishing iterations
};

/// OSQPソルバーラッパー（OSQP v1.0 API）
///
/// 使用例:
/// ```cpp
/// OSQPSolverWrapper solver;
/// solver.setup(P, q, A, l, u);
/// auto result = solver.solve();
/// if (result.converged) {
///     std::cout << "Solution: " << result.solution.transpose() << std::endl;
/// }
/// ```
class OSQPSolverWrapper {
public:
    OSQPSolverWrapper() : solver_(nullptr) {}

    ~OSQPSolverWrapper() {
        cleanup();
    }

    // コピー禁止
    OSQPSolverWrapper(const OSQPSolverWrapper&) = delete;
    OSQPSolverWrapper& operator=(const OSQPSolverWrapper&) = delete;

    /// QP問題を設定
    /// @param P 目的関数のヘッセ行列 (n×n, 半正定値, 上三角のみ使用)
    /// @param q 目的関数の線形項 (n×1)
    /// @param A 制約行列 (m×n)
    /// @param l 制約下限 (m×1)
    /// @param u 制約上限 (m×1)
    /// @param config OSQP設定パラメータ
    void setup(
        const Eigen::SparseMatrix<double>& P,
        const Eigen::VectorXd& q,
        const Eigen::SparseMatrix<double>& A,
        const Eigen::VectorXd& l,
        const Eigen::VectorXd& u,
        const OSQPConfig& config = OSQPConfig())
    {
        // 既存のソルバーをクリーンアップ
        cleanup();

        config_ = config;
        n_ = q.size();
        m_ = l.size();

        // P行列の上三角を抽出してCSC形式に変換
        auto P_upper = extract_upper_triangle(P);
        P_upper.makeCompressed();

        // AもCSC形式に変換
        Eigen::SparseMatrix<double> A_csc = A;
        A_csc.makeCompressed();

        // OSQPCscMatrix構造体を作成（スタック上、データはEigenから参照）
        OSQPCscMatrix P_osqp, A_osqp;

        // P行列データをコピーして保持（osqp_setupが内部コピーを作るまで有効）
        p_data_.assign(P_upper.valuePtr(), P_upper.valuePtr() + P_upper.nonZeros());
        p_row_idx_.resize(P_upper.nonZeros());
        p_col_ptr_.resize(P_upper.cols() + 1);
        for (int k = 0; k < P_upper.nonZeros(); ++k)
            p_row_idx_[k] = static_cast<OSQPInt>(P_upper.innerIndexPtr()[k]);
        for (int k = 0; k <= P_upper.cols(); ++k)
            p_col_ptr_[k] = static_cast<OSQPInt>(P_upper.outerIndexPtr()[k]);

        OSQPCscMatrix_set_data(&P_osqp, n_, n_,
                               static_cast<OSQPInt>(P_upper.nonZeros()),
                               p_data_.data(), p_row_idx_.data(), p_col_ptr_.data());

        // A行列データをコピーして保持
        a_data_.assign(A_csc.valuePtr(), A_csc.valuePtr() + A_csc.nonZeros());
        a_row_idx_.resize(A_csc.nonZeros());
        a_col_ptr_.resize(A_csc.cols() + 1);
        for (int k = 0; k < A_csc.nonZeros(); ++k)
            a_row_idx_[k] = static_cast<OSQPInt>(A_csc.innerIndexPtr()[k]);
        for (int k = 0; k <= A_csc.cols(); ++k)
            a_col_ptr_[k] = static_cast<OSQPInt>(A_csc.outerIndexPtr()[k]);

        OSQPCscMatrix_set_data(&A_osqp, m_, n_,
                               static_cast<OSQPInt>(A_csc.nonZeros()),
                               a_data_.data(), a_row_idx_.data(), a_col_ptr_.data());

        // q, l, u のデータをコピーして保持
        q_data_.assign(q.data(), q.data() + n_);
        l_data_.assign(l.data(), l.data() + m_);
        u_data_.assign(u.data(), u.data() + m_);

        // 設定を構成
        OSQPSettings* settings = OSQPSettings_new();
        if (!settings) {
            throw std::runtime_error("Failed to allocate OSQP settings");
        }
        settings->rho = config.rho;
        settings->sigma = config.sigma;
        settings->max_iter = config.max_iter;
        settings->eps_abs = config.eps_abs;
        settings->eps_rel = config.eps_rel;
        settings->alpha = config.alpha;
        settings->verbose = config.verbose ? 1 : 0;
        settings->warm_starting = config.warm_start ? 1 : 0;
        settings->polishing = config.polish ? 1 : 0;
        settings->polish_refine_iter = config.polish_refine_iter;

        // ソルバー初期化（v1.0 API）
        OSQPInt exitflag = osqp_setup(
            &solver_,
            &P_osqp, q_data_.data(),
            &A_osqp, l_data_.data(), u_data_.data(),
            m_, n_,
            settings
        );

        // 設定はosqp_setupが内部コピーを作るので解放
        OSQPSettings_free(settings);

        if (exitflag != 0) {
            solver_ = nullptr;
            throw std::runtime_error(
                "OSQP setup failed: " + std::string(osqp_error_message(exitflag)));
        }

        is_setup_ = true;
    }

    /// QP問題を解く
    /// @return 求解結果
    OSQPResult solve() {
        if (!is_setup_) {
            throw std::runtime_error("OSQP solver not setup. Call setup() first.");
        }

        auto start = std::chrono::high_resolution_clock::now();

        // 求解実行
        OSQPInt exitflag = osqp_solve(solver_);

        auto end = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration<double, std::milli>(end - start).count();

        if (exitflag != 0) {
            throw std::runtime_error(
                "OSQP solve failed: " + std::string(osqp_error_message(exitflag)));
        }

        // 結果を抽出（v1.0: solver_->solution, solver_->info）
        OSQPResult result;
        result.solution = Eigen::Map<Eigen::VectorXd>(solver_->solution->x, n_);
        result.dual_var = Eigen::Map<Eigen::VectorXd>(solver_->solution->y, m_);
        result.obj_val = solver_->info->obj_val;
        result.status = solver_->info->status_val;
        result.iter = solver_->info->iter;
        result.solve_time_ms = solve_time;
        result.converged = (result.status == OSQP_SOLVED ||
                           result.status == OSQP_SOLVED_INACCURATE);

        return result;
    }

    /// ウォームスタート（高速化、双対変数も設定）
    /// @param x_init 初期推定解
    /// @param y_init 初期双対変数
    void warm_start(const Eigen::VectorXd& x_init, const Eigen::VectorXd& y_init) {
        if (!is_setup_) {
            throw std::runtime_error("OSQP solver not setup.");
        }

        // v1.0: osqp_warm_start で x, y を同時設定
        osqp_warm_start(solver_,
                        const_cast<OSQPFloat*>(x_init.data()),
                        const_cast<OSQPFloat*>(y_init.data()));
    }

    /// ウォームスタート（高速化、primal変数のみ）
    /// @param x_init 初期推定解
    void warm_start(const Eigen::VectorXd& x_init) {
        if (!is_setup_) {
            throw std::runtime_error("OSQP solver not setup.");
        }

        // 双対変数はNULL → OSQPが内部で初期化
        osqp_warm_start(solver_,
                        const_cast<OSQPFloat*>(x_init.data()),
                        nullptr);
    }

    /// 線形項qを更新（高速再求解用）
    /// @param q_new 新しい線形項
    void update_q(const Eigen::VectorXd& q_new) {
        if (!is_setup_) {
            throw std::runtime_error("OSQP solver not setup.");
        }

        // v1.0: osqp_update_data_vec(solver, q_new, l_new, u_new)
        // q のみ更新 → l, u は NULL
        q_data_.assign(q_new.data(), q_new.data() + n_);
        osqp_update_data_vec(solver_, q_data_.data(), nullptr, nullptr);
    }

    /// 制約境界を更新
    /// @param l_new 新しい下限
    /// @param u_new 新しい上限
    void update_bounds(const Eigen::VectorXd& l_new, const Eigen::VectorXd& u_new) {
        if (!is_setup_) {
            throw std::runtime_error("OSQP solver not setup.");
        }

        // v1.0: osqp_update_data_vec(solver, q, l, u)
        // l, u のみ更新 → q は NULL
        l_data_.assign(l_new.data(), l_new.data() + m_);
        u_data_.assign(u_new.data(), u_new.data() + m_);
        osqp_update_data_vec(solver_, nullptr, l_data_.data(), u_data_.data());
    }

    /// ワークスペースが設定済みか
    bool is_ready() const { return is_setup_; }

private:
    OSQPSolver* solver_;           ///< v1.0: OSQPSolver（OSQPWorkspaceの代わり）
    OSQPConfig config_;
    OSQPInt n_ = 0;                ///< 変数数
    OSQPInt m_ = 0;                ///< 制約数
    bool is_setup_ = false;

    // データバッファ（osqp_setupに渡すデータの寿命管理）
    std::vector<OSQPFloat> p_data_, a_data_;
    std::vector<OSQPInt> p_row_idx_, p_col_ptr_;
    std::vector<OSQPInt> a_row_idx_, a_col_ptr_;
    std::vector<OSQPFloat> q_data_, l_data_, u_data_;

    /// クリーンアップ
    void cleanup() {
        if (solver_) {
            osqp_cleanup(solver_);  // v1.0: 全メモリを解放
            solver_ = nullptr;
        }
        is_setup_ = false;
    }

    /// 上三角行列を抽出（OSQPはPの上三角のみ使用）
    Eigen::SparseMatrix<double> extract_upper_triangle(const Eigen::SparseMatrix<double>& mat) {
        Eigen::SparseMatrix<double> upper(mat.rows(), mat.cols());
        std::vector<Eigen::Triplet<double>> triplets;

        for (int k = 0; k < mat.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
                if (it.row() <= it.col()) {  // 上三角のみ
                    triplets.emplace_back(it.row(), it.col(), it.value());
                }
            }
        }

        upper.setFromTriplets(triplets.begin(), triplets.end());
        return upper;
    }
};

// 後方互換性のためのエイリアス
using OSQPSolver = OSQPSolverWrapper;

} // namespace fsae::control

#else
// OSQPが利用不可の場合はダミー実装
#include <stdexcept>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace fsae::control {
    struct OSQPResult {
        Eigen::VectorXd solution;
        Eigen::VectorXd dual_var;
        double obj_val = 0;
        int status = 0;
        int iter = 0;
        double solve_time_ms = 0;
        bool converged = false;
        std::string status_message() const { return "OSQP not available"; }
    };
    struct OSQPConfig {};
    class OSQPSolverWrapper {
    public:
        void setup(const Eigen::SparseMatrix<double>&, const Eigen::VectorXd&,
                   const Eigen::SparseMatrix<double>&, const Eigen::VectorXd&,
                   const Eigen::VectorXd&, const OSQPConfig& = OSQPConfig()) {
            throw std::runtime_error("OSQP not compiled. Install OSQP and rebuild with -DHAS_OSQP=1.");
        }
        OSQPResult solve() { throw std::runtime_error("OSQP not available"); }
        void warm_start(const Eigen::VectorXd&, const Eigen::VectorXd&) {
            throw std::runtime_error("OSQP not available");
        }
        void warm_start(const Eigen::VectorXd&) {
            throw std::runtime_error("OSQP not available");
        }
        void update_q(const Eigen::VectorXd&) { throw std::runtime_error("OSQP not available"); }
        void update_bounds(const Eigen::VectorXd&, const Eigen::VectorXd&) {
            throw std::runtime_error("OSQP not available");
        }
        bool is_ready() const { return false; }
    };
    using OSQPSolver = OSQPSolverWrapper;
}
#endif // HAS_OSQP
