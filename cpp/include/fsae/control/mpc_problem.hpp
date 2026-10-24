#pragma once
/// @file mpc_problem.hpp
/// @brief MPC最適化問題のQP定式化
///
/// Phase 8a: BicycleModelの逐次線形化 + OSQP形式のQP構築
///
/// 決定変数: z = [x₀, u₀, x₁, u₁, ..., x_N]
///   x_k ∈ R^6 (x, y, ψ, vx, vy, r)
///   u_k ∈ R^2 (δ, ax)
///
/// コスト:  Σ w_lat·(x-xref)² + w_heading·(ψ-ψref)² + ...
/// 制約:    x_{k+1} = A_k·x_k + B_k·u_k + c_k  (動力学等式)
///          u_min ≤ u_k ≤ u_max                  (入力バウンド)
///          |δ_k - δ_{k-1}| ≤ Δδ_max             (レート制約)

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>
#include <vector>
#include <cmath>

#include "../vehicle/bicycle_model.hpp"
#include "../track/track_distance.hpp"

namespace fsae::control {

/// MPC設定パラメータ
struct MPCConfig {
    // ホライズン
    int N = 20;             ///< 予測ステップ数
    double dt = 0.025;      ///< 制御周期 [s] (40Hz)

    // コスト重み
    double w_x = 5.0;       ///< X位置追従
    double w_y = 5.0;       ///< Y位置追従
    double w_psi = 2.0;     ///< ヘディング追従
    double w_vx = 0.5;      ///< 速度追従
    double w_vy = 0.0;      ///< 横速度ペナルティ
    double w_r = 0.0;       ///< ヨーレートペナルティ

    double w_delta = 0.01;      ///< ステアリング入力ペナルティ
    double w_ax = 0.01;         ///< 加速度入力ペナルティ
    double w_delta_rate = 0.1;  ///< ステアリングレートペナルティ
    double w_ax_rate = 0.01;    ///< 加速度レートペナルティ

    // 終端コスト倍率
    double terminal_weight = 3.0;

    // Progress objective (disabled by default for legacy MPC)
    double terminal_progress_weight = 0.0; ///< penalty for terminal progress deficit
    double progress_target_fraction = 0.85; ///< fraction of feasible speed envelope to require

    // 入力制約
    double delta_max = 0.35;    ///< 最大舵角 [rad] (±20度)
    double ax_max = 10.0;       ///< 最大加速度 [m/s²]
    double ax_min = -15.0;      ///< 最大減速度 [m/s²]

    // レート制約
    double delta_rate_max = 3.14 * 0.025;  ///< 最大舵角変化 [rad/step]

    // 速度制約
    double vx_min = 2.0;    ///< 最小速度 [m/s]
    double vx_max = 30.0;   ///< 最大速度 [m/s]

    // Track-boundary speed envelope (disabled by default for legacy MPC)
    bool use_track_margin_speed_cap = false;
    double track_margin_speed_cap = 8.0;       ///< speed cap near track boundary [m/s]
    double track_margin_speed_release = 1.0;   ///< boundary margin that releases speed cap [m]
    double hard_boundary_margin = 0.30;        ///< hard safety margin for high penalty [m]
    double soft_boundary_margin = 0.85;        ///< soft safety margin for low penalty [m]
    double hard_boundary_weight = 1000.0;
    double soft_boundary_weight = 80.0;
    double outward_rate_weight = 40.0;

    // アクチュエータモデル
    /// 予測ロールアウト内で閉ループ操舵アクチュエータの2次応答を模擬するか
    /// （torque_mode 時のみ有効）
    bool use_actuator_dynamics_in_prediction = true;
    double steering_inner_loop_omega_n = 30.0; ///< 閉ループ操舵帯域 [rad/s]
    double steering_inner_loop_zeta = 0.70;    ///< 閉ループ操舵減衰比 [-]

    // Attack driver fixed-course speed planner knobs.
    // These are ignored by legacy pid/mpc/mpc_v2/rl paths.
    double attack_speed_lateral_safety = 0.70;
    double attack_speed_braking_safety = 0.78;
    double attack_speed_accel_limit = 5.5;
    double attack_rollout_understeer_gradient = 0.0; ///< [rad/(m/s^2)] for attack yaw-rate prediction
    double attack_floor_straight = 0.92;
    double attack_floor_exit = 0.92;
    double attack_floor_complex = 0.86;
    double attack_floor_turn_in = 0.84;
    double attack_floor_apex = 0.84;
    double attack_floor_brake = 0.78;
    double attack_braking_corridor_decel_scale = 1.25;
    double attack_braking_corridor_band_scale = 1.25;
    double attack_braking_corridor_mild_accel_scale = 1.25;
    double attack_braking_corridor_overspeed_brake_gain = 1.10;
    bool attack_braking_corridor_guard_only = true;
    std::vector<std::array<double, 3>> attack_speed_caps; ///< local caps as {s [m], speed [m/s], radius [m]}
    std::vector<std::array<double, 3>> attack_speed_windows; ///< local window caps as {s_start [m], s_end [m], speed [m/s]}
};

/// QP問題データ（OSQP形式）
struct QPData {
    Eigen::SparseMatrix<double> P;  ///< ヘッセ行列 (nz × nz)
    Eigen::VectorXd q;              ///< 線形項 (nz)
    Eigen::SparseMatrix<double> A;  ///< 制約行列 (nc × nz)
    Eigen::VectorXd l;              ///< 制約下限 (nc)
    Eigen::VectorXd u;              ///< 制約上限 (nc)
    int n_vars;                     ///< 変数数
    int n_constraints;              ///< 制約数
};

/// MPC QP問題定式化
class MPCProblem {
public:
    static constexpr int NX = 6;  ///< 状態次元
    static constexpr int NU = 2;  ///< 制御次元

    explicit MPCProblem(const MPCConfig& config = MPCConfig())
        : config_(config)
    {}

    /// QP問題を定式化
    /// @param x0 現在状態
    /// @param ref_traj 参照軌道 (N+1点)
    /// @param prev_delta 前回の舵角 [rad]
    /// @param model 自転車モデル（線形化用）
    /// @return QPData (P, q, A, l, u)
    QPData formulate(
        const vehicle::BicycleState6DOF& x0,
        const std::vector<track::ReferencePoint>& ref_traj,
        double prev_delta,
        const vehicle::BicycleModel6DOF& model) const
    {
        const int N = config_.N;
        // 決定変数: [x₀, u₀, x₁, u₁, ..., u_{N-1}, x_N]
        // = (N+1)*NX + N*NU
        const int nz = (N + 1) * NX + N * NU;

        // 制約:
        //   N*NX     動力学等式 (x_{k+1} = A_k*x_k + B_k*u_k + c_k)
        //   NX       初期状態等式 (x₀ = x_current)
        //   N*NU     入力バウンド
        //   (N-1)*1  舵角レート制約 (隣接ステップ間)
        //   N*1      速度制約 (vx_min ≤ vx_k ≤ vx_max)
        const int nc_dynamics = N * NX;
        const int nc_initial = NX;
        const int nc_input = N * NU;
        const int nc_delta_rate = (N > 1) ? (N - 1) : 0;
        const int nc_vx = N;  // x_1 ... x_N のvx
        const int nc = nc_dynamics + nc_initial + nc_input + nc_delta_rate + nc_vx;

        // === P行列（コスト二次項）とq（コスト線形項）を構築 ===
        std::vector<Eigen::Triplet<double>> P_triplets;
        Eigen::VectorXd q = Eigen::VectorXd::Zero(nz);

        // 状態コスト重みベクトル
        Eigen::VectorXd Qw(NX);
        Qw << config_.w_x, config_.w_y, config_.w_psi,
               config_.w_vx, config_.w_vy, config_.w_r;

        // 制御コスト重みベクトル
        Eigen::VectorXd Rw(NU);
        Rw << config_.w_delta, config_.w_ax;

        for (int k = 0; k <= N; ++k) {
            int x_offset = var_x_offset(k);
            double weight_mult = (k == N) ? config_.terminal_weight : 1.0;

            // 参照軌道からの偏差コスト: w_i * (x_i - x_ref_i)²
            // = w_i * x_i² - 2*w_i*x_ref_i*x_i + const
            const auto& ref = (k < static_cast<int>(ref_traj.size()))
                ? ref_traj[k] : ref_traj.back();

            Eigen::VectorXd x_ref(NX);
            x_ref << ref.x, ref.y, ref.psi, ref.vx, 0.0, 0.0;

            for (int i = 0; i < NX; ++i) {
                double w = Qw(i) * weight_mult;
                if (w > 0) {
                    P_triplets.emplace_back(x_offset + i, x_offset + i, w);
                    q(x_offset + i) = -w * x_ref(i);
                }
            }

            // 制御コスト
            if (k < N) {
                int u_offset = var_u_offset(k);
                for (int i = 0; i < NU; ++i) {
                    if (Rw(i) > 0) {
                        P_triplets.emplace_back(u_offset + i, u_offset + i, Rw(i));
                    }
                }
            }
        }

        // 舵角レートコスト: w_delta_rate * (δ_k - δ_{k-1})²
        // = w * (δ_k² - 2*δ_k*δ_{k-1} + δ_{k-1}²)
        if (config_.w_delta_rate > 0) {
            double w = config_.w_delta_rate;

            // k=0: (δ₀ - prev_delta)²
            {
                int u0_off = var_u_offset(0);
                P_triplets.emplace_back(u0_off, u0_off, w);  // δ₀²
                q(u0_off) += -w * prev_delta;                 // -2*w*prev*δ₀ (の半分はPから)
            }

            // k=1..N-1: (δ_k - δ_{k-1})²
            for (int k = 1; k < N; ++k) {
                int uk_off = var_u_offset(k);
                int uk1_off = var_u_offset(k - 1);

                P_triplets.emplace_back(uk_off, uk_off, w);
                P_triplets.emplace_back(uk1_off, uk1_off, w);
                // クロス項: -2*w*δ_k*δ_{k-1} → P上三角に-w
                if (uk1_off < uk_off) {
                    P_triplets.emplace_back(uk1_off, uk_off, -w);
                } else {
                    P_triplets.emplace_back(uk_off, uk1_off, -w);
                }
            }
        }

        // ax レートコスト
        if (config_.w_ax_rate > 0) {
            double w = config_.w_ax_rate;
            for (int k = 1; k < N; ++k) {
                int uk_off = var_u_offset(k) + 1;
                int uk1_off = var_u_offset(k - 1) + 1;
                P_triplets.emplace_back(uk_off, uk_off, w);
                P_triplets.emplace_back(uk1_off, uk1_off, w);
                if (uk1_off < uk_off) {
                    P_triplets.emplace_back(uk1_off, uk_off, -w);
                } else {
                    P_triplets.emplace_back(uk_off, uk1_off, -w);
                }
            }
        }

        // === A行列と l/u（制約）を構築 ===
        std::vector<Eigen::Triplet<double>> A_triplets;
        Eigen::VectorXd l_vec = Eigen::VectorXd::Zero(nc);
        Eigen::VectorXd u_vec = Eigen::VectorXd::Zero(nc);

        int row = 0;

        // --- 1. 初期状態等式: x₀ = x_current ---
        {
            Eigen::VectorXd x0_vec = x0.to_vector();
            int x_off = var_x_offset(0);
            for (int i = 0; i < NX; ++i) {
                A_triplets.emplace_back(row + i, x_off + i, 1.0);
                l_vec(row + i) = x0_vec(i);
                u_vec(row + i) = x0_vec(i);
            }
            row += NX;
        }

        // --- 2. 動力学等式: x_{k+1} = A_k*x_k + B_k*u_k + c_k ---
        // 各ステップで基準軌道まわりに線形化
        // x_{k+1} = f(x_ref, u_ref) + A*(x_k - x_ref) + B*(u_k - u_ref)
        //         = A*x_k + B*u_k + c_k
        // c_k = f(x_ref, u_ref) - A*x_ref - B*u_ref
        {
            // 基準軌道を構築（前回の予測か参照軌道）
            vehicle::BicycleState6DOF x_lin = x0;
            vehicle::BicycleControl u_lin{0.0, 0.0};

            for (int k = 0; k < N; ++k) {
                // 線形化点の制御入力を参照から推定
                if (k < static_cast<int>(ref_traj.size()) - 1) {
                    // 参照のヘディング変化から概略ステアリング推定
                    u_lin.delta = 0.0;  // 初期は0（逐次改善で更新される）
                    u_lin.ax = 0.0;
                }

                // 線形化
                auto [Ak, Bk] = model.linearize(x_lin, u_lin, config_.dt);

                // 基準次状態
                auto x_next_nom = model.predict_step(x_lin, u_lin, config_.dt);
                Eigen::VectorXd x_nom_vec = x_next_nom.to_vector();
                Eigen::VectorXd x_lin_vec = x_lin.to_vector();
                Eigen::VectorXd u_lin_vec = u_lin.to_vector();

                // c_k = x_nom - A*x_lin - B*u_lin
                Eigen::VectorXd ck = x_nom_vec - Ak * x_lin_vec - Bk * u_lin_vec;

                // 制約: x_{k+1} - A_k*x_k - B_k*u_k = c_k
                // → -A_k*x_k - B_k*u_k + I*x_{k+1} = c_k
                int xk_off = var_x_offset(k);
                int uk_off = var_u_offset(k);
                int xk1_off = var_x_offset(k + 1);

                for (int i = 0; i < NX; ++i) {
                    // -A_k * x_k
                    for (int j = 0; j < NX; ++j) {
                        if (std::abs(Ak(i, j)) > 1e-12) {
                            A_triplets.emplace_back(row + i, xk_off + j, -Ak(i, j));
                        }
                    }
                    // -B_k * u_k
                    for (int j = 0; j < NU; ++j) {
                        if (std::abs(Bk(i, j)) > 1e-12) {
                            A_triplets.emplace_back(row + i, uk_off + j, -Bk(i, j));
                        }
                    }
                    // I * x_{k+1}
                    A_triplets.emplace_back(row + i, xk1_off + i, 1.0);

                    // l = u = c_k
                    l_vec(row + i) = ck(i);
                    u_vec(row + i) = ck(i);
                }
                row += NX;

                // 次の線形化点を更新（参照軌道ベース）
                if (k + 1 < static_cast<int>(ref_traj.size())) {
                    const auto& ref_next = ref_traj[k + 1];
                    x_lin = vehicle::BicycleState6DOF{
                        ref_next.x, ref_next.y, ref_next.psi,
                        ref_next.vx, 0.0, ref_next.vx * ref_next.kappa
                    };
                }
            }
        }

        // --- 3. 入力バウンド: u_min ≤ u_k ≤ u_max ---
        for (int k = 0; k < N; ++k) {
            int u_off = var_u_offset(k);
            // δ制約
            A_triplets.emplace_back(row, u_off, 1.0);
            l_vec(row) = -config_.delta_max;
            u_vec(row) = config_.delta_max;
            row++;
            // ax制約
            A_triplets.emplace_back(row, u_off + 1, 1.0);
            l_vec(row) = config_.ax_min;
            u_vec(row) = config_.ax_max;
            row++;
        }

        // --- 4. 舵角レート制約: |δ_k - δ_{k-1}| ≤ Δδ_max ---
        for (int k = 1; k < N; ++k) {
            int uk_off = var_u_offset(k);
            int uk1_off = var_u_offset(k - 1);
            // δ_k - δ_{k-1}
            A_triplets.emplace_back(row, uk_off, 1.0);
            A_triplets.emplace_back(row, uk1_off, -1.0);
            l_vec(row) = -config_.delta_rate_max;
            u_vec(row) = config_.delta_rate_max;
            row++;
        }

        // --- 5. 速度制約: vx_min ≤ vx_k ≤ vx_max (k=1..N) ---
        for (int k = 1; k <= N; ++k) {
            int x_off = var_x_offset(k);
            A_triplets.emplace_back(row, x_off + 3, 1.0);  // vx is index 3
            l_vec(row) = config_.vx_min;
            u_vec(row) = config_.vx_max;
            row++;
        }

        // === 組み立て ===
        Eigen::SparseMatrix<double> P_sparse(nz, nz);
        P_sparse.setFromTriplets(P_triplets.begin(), P_triplets.end());

        Eigen::SparseMatrix<double> A_sparse(nc, nz);
        A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());

        return {P_sparse, q, A_sparse, l_vec, u_vec, nz, nc};
    }

    /// QP解から制御入力を抽出
    /// @param solution QP解ベクトル
    /// @return (delta, ax) のペア
    std::pair<double, double> extract_control(
        const Eigen::VectorXd& solution) const
    {
        int u0_off = var_u_offset(0);
        return {solution(u0_off), solution(u0_off + 1)};
    }

    /// QP解から予測状態軌道を抽出
    std::vector<vehicle::BicycleState6DOF> extract_trajectory(
        const Eigen::VectorXd& solution) const
    {
        std::vector<vehicle::BicycleState6DOF> traj;
        for (int k = 0; k <= config_.N; ++k) {
            int x_off = var_x_offset(k);
            Eigen::VectorXd xk = solution.segment(x_off, NX);
            traj.push_back(vehicle::BicycleState6DOF::from_vector(xk));
        }
        return traj;
    }

    /// 変数数を取得
    int n_vars() const { return (config_.N + 1) * NX + config_.N * NU; }

    /// 設定を取得
    const MPCConfig& config() const { return config_; }

private:
    /// 状態変数x_kの開始オフセット
    /// レイアウト: [x₀, u₀, x₁, u₁, ..., u_{N-1}, x_N]
    int var_x_offset(int k) const {
        return k * (NX + NU);
    }

    /// 制御変数u_kの開始オフセット
    int var_u_offset(int k) const {
        return k * (NX + NU) + NX;
    }

    MPCConfig config_;
};

} // namespace fsae::control
