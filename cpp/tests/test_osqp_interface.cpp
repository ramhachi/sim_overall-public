/// @file test_osqp_interface.cpp
/// @brief OSQP interfaceのユニットテスト（Phase 8a, OSQP v1.0）

#include <gtest/gtest.h>

#ifdef HAS_OSQP
#include "fsae/control/osqp_interface.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <limits>

using namespace fsae::control;

/// テスト用フィクスチャ
class OSQPInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        solver_ = std::make_unique<OSQPSolverWrapper>();
    }

    std::unique_ptr<OSQPSolverWrapper> solver_;
};

/// テスト1: 簡単なQP問題（OSQP公式デモと同じ）
/// minimize  0.5·x'·[4 1; 1 2]·x + [1 1]·x
/// s.t.      1 <= x1 + x2 <= 1   (等式制約)
///           0 <= x1 <= 0.7
///           0 <= x2 <= 0.7
/// 解析解: x* ≈ [0.3, 0.7]
TEST_F(OSQPInterfaceTest, SimpleQP) {
    // 問題サイズ
    int n = 2;  // 変数数
    int m = 3;  // 制約数

    // P行列（ヘッセ行列）- 上三角のみ指定
    Eigen::SparseMatrix<double> P(n, n);
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.emplace_back(0, 0, 4.0);
    P_triplets.emplace_back(0, 1, 1.0);
    P_triplets.emplace_back(1, 1, 2.0);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    // q（線形項）
    Eigen::VectorXd q(n);
    q << 1.0, 1.0;

    // A（制約行列）: 3行2列
    Eigen::SparseMatrix<double> A(m, n);
    std::vector<Eigen::Triplet<double>> A_triplets;
    // 制約1: x1 + x2 = 1（等式制約: l=u=1）
    A_triplets.emplace_back(0, 0, 1.0);
    A_triplets.emplace_back(0, 1, 1.0);
    // 制約2: 0 <= x1 <= 0.7
    A_triplets.emplace_back(1, 0, 1.0);
    // 制約3: 0 <= x2 <= 0.7
    A_triplets.emplace_back(2, 1, 1.0);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // 制約境界: l <= Ax <= u
    Eigen::VectorXd l(m);
    l << 1.0, 0.0, 0.0;
    Eigen::VectorXd u(m);
    u << 1.0, 0.7, 0.7;

    // 求解
    solver_->setup(P, q, A, l, u);
    auto result = solver_->solve();

    // 検証
    EXPECT_TRUE(result.converged) << "Status: " << result.status_message();
    EXPECT_NEAR(result.solution(0), 0.3, 0.01);
    EXPECT_NEAR(result.solution(1), 0.7, 0.01);
    EXPECT_LT(result.solve_time_ms, 100.0);  // 100ms以内
    std::cout << "Solution: [" << result.solution.transpose() << "]" << std::endl;
    std::cout << "Solve time: " << result.solve_time_ms << " ms" << std::endl;
}

/// テスト2: 無制約QP問題
/// minimize  0.5·x'·[2 0; 0 2]·x + [-2 -2]·x
/// 解析解: x* = [1, 1]
TEST_F(OSQPInterfaceTest, UnconstrainedQP) {
    int n = 2;
    int m = 2;  // OSQP v1.0 は m >= 1 が必要、ダミー制約を追加

    // P = 2·I
    Eigen::SparseMatrix<double> P(n, n);
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.emplace_back(0, 0, 2.0);
    P_triplets.emplace_back(1, 1, 2.0);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    // q = [-2, -2]
    Eigen::VectorXd q(n);
    q << -2.0, -2.0;

    // ダミー制約: -inf <= x <= inf（実質無制約）
    Eigen::SparseMatrix<double> A(m, n);
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.emplace_back(0, 0, 1.0);
    A_triplets.emplace_back(1, 1, 1.0);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    constexpr double inf = 1e30;
    Eigen::VectorXd l(m);
    l << -inf, -inf;
    Eigen::VectorXd u(m);
    u << inf, inf;

    solver_->setup(P, q, A, l, u);
    auto result = solver_->solve();

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.solution(0), 1.0, 0.01);
    EXPECT_NEAR(result.solution(1), 1.0, 0.01);
}

/// テスト3: ウォームスタート
TEST_F(OSQPInterfaceTest, WarmStart) {
    // 同じQP問題を2回解く（2回目はウォームスタート）
    int n = 2;
    int m = 2;

    Eigen::SparseMatrix<double> P(n, n);
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.emplace_back(0, 0, 4.0);
    P_triplets.emplace_back(1, 1, 4.0);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    Eigen::VectorXd q(n);
    q << -1.0, -1.0;

    Eigen::SparseMatrix<double> A(m, n);
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.emplace_back(0, 0, 1.0);
    A_triplets.emplace_back(1, 1, 1.0);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    Eigen::VectorXd l(m);
    l << 0.0, 0.0;
    Eigen::VectorXd u(m);
    u << 1.0, 1.0;

    // 1回目: コールドスタート
    solver_->setup(P, q, A, l, u);
    auto result1 = solver_->solve();

    // 2回目: ウォームスタート
    solver_->warm_start(result1.solution, result1.dual_var);
    auto result2 = solver_->solve();

    // ウォームスタートの方が速い（または同等）
    EXPECT_LE(result2.iter, result1.iter);
    std::cout << "Cold start: " << result1.iter << " iter, "
              << result1.solve_time_ms << " ms" << std::endl;
    std::cout << "Warm start: " << result2.iter << " iter, "
              << result2.solve_time_ms << " ms" << std::endl;
}

/// テスト4: 実行不可能問題
TEST_F(OSQPInterfaceTest, InfeasibleProblem) {
    int n = 1;
    int m = 2;

    Eigen::SparseMatrix<double> P(n, n);
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.emplace_back(0, 0, 1.0);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    Eigen::VectorXd q(n);
    q << 0.0;

    Eigen::SparseMatrix<double> A(m, n);
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.emplace_back(0, 0, 1.0);
    A_triplets.emplace_back(1, 0, 1.0);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    // 矛盾する制約: x >= 2 AND x <= 1
    constexpr double inf = 1e30;
    Eigen::VectorXd l(m);
    l << 2.0, -inf;
    Eigen::VectorXd u(m);
    u << inf, 1.0;

    solver_->setup(P, q, A, l, u);
    auto result = solver_->solve();

    // 実行不可能と判定される
    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.status, OSQP_PRIMAL_INFEASIBLE);
    std::cout << "Infeasible status: " << result.status_message() << std::endl;
}

/// テスト5: パラメータ更新
TEST_F(OSQPInterfaceTest, ParameterUpdate) {
    int n = 2;
    int m = 2;

    Eigen::SparseMatrix<double> P(n, n);
    std::vector<Eigen::Triplet<double>> P_triplets;
    P_triplets.emplace_back(0, 0, 2.0);
    P_triplets.emplace_back(1, 1, 2.0);
    P.setFromTriplets(P_triplets.begin(), P_triplets.end());

    Eigen::VectorXd q(n);
    q << -1.0, -1.0;

    Eigen::SparseMatrix<double> A(m, n);
    std::vector<Eigen::Triplet<double>> A_triplets;
    A_triplets.emplace_back(0, 0, 1.0);
    A_triplets.emplace_back(1, 1, 1.0);
    A.setFromTriplets(A_triplets.begin(), A_triplets.end());

    Eigen::VectorXd l(m);
    l << 0.0, 0.0;
    Eigen::VectorXd u(m);
    u << 1.0, 1.0;

    solver_->setup(P, q, A, l, u);
    auto result1 = solver_->solve();

    // qを更新
    Eigen::VectorXd q_new(n);
    q_new << -2.0, -2.0;
    solver_->update_q(q_new);

    auto result2 = solver_->solve();

    // 解は変わる（より大きな負の勾配）
    EXPECT_TRUE(result2.converged);
    // 解は境界に張り付く
    EXPECT_NEAR(result2.solution(0), 1.0, 0.01);
    EXPECT_NEAR(result2.solution(1), 1.0, 0.01);
}

#else
// OSQPが無効の場合のダミーテスト
TEST(OSQPInterfaceTest, NotAvailable) {
    GTEST_SKIP() << "OSQP not compiled. Install OSQP and rebuild.";
}
#endif // HAS_OSQP
