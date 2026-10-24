# `cpp/tests/test_osqp_interface.cpp`

## 1. ファイルの目的

OSQP ラッパー `OSQPSolverWrapper` が簡単な QP、無制約問題、ウォームスタート、実行不可能判定、パラメータ更新に対応できるかを確認するテストです。

## 2. 実行上の役割

- MPC の下位レイヤである QP ソルバ層の健全性確認を担います。
- `HAS_OSQP` が無効な環境では明示的にスキップし、ビルド構成依存であることを示します。

## 3. 依存関係

- `fsae/control/osqp_interface.hpp`（`HAS_OSQP` 時）
  - `OSQPSolverWrapper`
  - 結果構造体の `solution`, `dual_var`, `iter`, `status`, `status_message()`
- `Eigen/Dense`, `Eigen/Sparse`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - QP の `P`, `q`, `A`, `l`, `u`
  - ウォームスタート初期値
  - 更新後 `q`
- 出力
  - 収束可否
  - 解ベクトル
  - 反復回数、求解時間、ステータス

## 5. 関数・メソッド・クラスの網羅的な説明

### `class OSQPInterfaceTest : public ::testing::Test`（`HAS_OSQP`）

- `OSQPSolverWrapper` 共通生成を行うフィクスチャです。

#### `void SetUp() override`

- テストごとに新しい `OSQPSolverWrapper` を生成します。

#### メンバ

- `std::unique_ptr<OSQPSolverWrapper> solver_`

### `TEST_F(OSQPInterfaceTest, SimpleQP)`（`HAS_OSQP`）

- 公式デモに近い 2 変数 QP を解き、解析解 `[0.3, 0.7]` 近傍へ収束することを確認します。

### `TEST_F(OSQPInterfaceTest, UnconstrainedQP)`（`HAS_OSQP`）

- 実質無制約な 2 変数問題を解き、最適解 `[1, 1]` を得ることを確認します。

### `TEST_F(OSQPInterfaceTest, WarmStart)`（`HAS_OSQP`）

- 同じ問題をコールドスタート / ウォームスタートで 2 回解き、ウォームスタートの反復回数が悪化しないことを確認します。

### `TEST_F(OSQPInterfaceTest, InfeasibleProblem)`（`HAS_OSQP`）

- 矛盾制約 `x >= 2` かつ `x <= 1` を与え、`OSQP_PRIMAL_INFEASIBLE` が返ることを確認します。

### `TEST_F(OSQPInterfaceTest, ParameterUpdate)`（`HAS_OSQP`）

- 一度セットアップした問題の線形項 `q` を更新し、解が新しい最適点へ変わることを確認します。

### `TEST(OSQPInterfaceTest, NotAvailable)`（`#else` 側）

- OSQP 非対応ビルドでは `GTEST_SKIP()` を返すだけのダミーテストです。
- 「失敗ではなく未利用構成」であることをテスト結果に明示します。
