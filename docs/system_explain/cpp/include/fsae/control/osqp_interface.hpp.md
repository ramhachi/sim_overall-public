# osqp_interface.hpp

## 1. ファイルの目的
OSQP v1.0 API を Eigen ベースの C++ コードから扱うためのラッパーです。

## 2. 実行上の役割
MPC 定式化結果を実際に解く層です。`HAS_OSQP` 有効時は実ソルバー、無効時は例外を投げるダミー実装になります。

## 3. 依存関係
- `control/mpc_problem*.hpp`
- `driver/mpc_driver*.hpp`

## 4. 入力/出力
- 入力: `P`, `q`, `A`, `l`, `u`, `OSQPConfig`
- 出力: `OSQPResult`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct OSQPResult`
`solution`, `dual_var`, `obj_val`, `status`, `iter`, `solve_time_ms`, `converged` を持ち、`status_message()` で状態文字列を返します。

### `struct OSQPConfig`
`rho`, `sigma`, `max_iter`, `eps_abs`, `eps_rel`, `alpha`, `verbose`, `warm_start`, `polish`, `polish_refine_iter` を持つ設定です。

### `class OSQPSolverWrapper`
- `OSQPSolverWrapper()`, `~OSQPSolverWrapper()`
- コピー/代入削除
- `setup(...)`: 上三角抽出と CSC 化を行って `osqp_setup()` を実行
- `solve()`
- `warm_start(x,y)`, `warm_start(x)`
- `update_q(...)`, `update_bounds(...)`
- `is_ready()`
- `cleanup()`
- `extract_upper_triangle(...)`

### `using OSQPSolver = OSQPSolverWrapper`
後方互換エイリアスです。

### OSQP 無効時のダミー実装
`OSQPResult`, `OSQPConfig`, `OSQPSolverWrapper`, `OSQPSolver` を最小実装で定義し、使用時は例外または `false` を返します。
