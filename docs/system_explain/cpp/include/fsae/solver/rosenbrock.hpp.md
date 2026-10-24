# rosenbrock.hpp

## ファイルの目的
- 剛性 ODE 向けの Rosenbrock23 法ソルバを提供します。

## 実行上の役割
- 状態微分関数とヤコビアン関数を受け取り、適応タイムステップで時間積分します。
- 固定次元テンプレート版と `Eigen::VectorXd` 動的次元版を両方備えます。

## 依存関係
- 外部: `Eigen/Dense`, `Eigen/LU`, `<functional>`, `<cmath>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: 微分関数 `f(t,x)`, ヤコビアン `J(t,x)`, 初期状態、積分区間、設定
- 出力: 次状態、`SolverResult`

## 関数・メソッド・クラスの網羅的な説明
- `struct Rosenbrock23Coeffs`
  - `gamma`, `a21`, `c21`, `b1`, `b2`, `e1`, `e2` を定数として持つ係数表です。
- `struct RosenbrockConfig`
  - `dt_init`, `dt_min`, `dt_max`, `atol`, `rtol`, `max_iterations` を持ちます。
- `struct SolverResult`
  - `success`, `steps`, `final_time`, `jacobian_evals` を返します。
- `template<int N> class Rosenbrock23`
  - 型別名 `StateVector`, `JacobianMatrix`, `DerivFunc`, `JacobianFunc` を提供します。
  - `Rosenbrock23(DerivFunc f, JacobianFunc J, RosenbrockConfig config = {})`
    - 微分関数・ヤコビアン関数・設定を保持します。
  - `step(double t, const StateVector& x, double dt)`
    - `W = I - gamma*dt*J` を LU 分解し、2 段の Rosenbrock23 ステージから `x_new` を返します。
    - 誤差推定は `e1*k1 + e2*k2` のノルムで内部保存します。
  - `integrate(double t0, double t_end, StateVector& x)`
    - `step()` を繰り返し、誤差比 `err_ratio` に基づきステップ受理/拒否と `dt` 調整を行います。
    - 許容誤差は `atol + rtol * ||x||` です。
  - `jacobian_count() const`
    - 直近積分でのヤコビアン評価回数を返します。
  - private `f_`, `J_`, `config_`, `error_estimate_`, `jacobian_count_`
    - 実行に必要な保持状態です。
- `class RosenbrockDynamic`
  - 動的次元版で、公開 API とアルゴリズムは `Rosenbrock23<N>` と同じです。
  - `StateVector=Eigen::VectorXd`, `JacobianMatrix=Eigen::MatrixXd` を使う点だけが異なります。
  - `step()`, `integrate()`, `jacobian_count()`、private メンバ構成も固定次元版と対応しています。
