# mpc_problem.hpp

## 1. ファイルの目的
6自由度自転車モデル用 MPC を、OSQP が解ける QP 形式へ定式化するヘッダです。参照軌道追従・入力制約・舵角レート制約・速度制約をまとめて扱います。

## 2. 実行上の役割
`driver/mpc_driver.hpp` が毎周期呼び出し、`BicycleModel6DOF` を線形化しながら予測ホライズン全体のコスト行列と制約行列を構築します。

## 3. 依存関係
- `vehicle/bicycle_model.hpp`
- `track/track_distance.hpp`
- `driver/mpc_driver.hpp`
- `control/osqp_interface.hpp`

## 4. 入力/出力
- 入力: `x0`, `ref_traj`, `prev_delta`, `model`
- 出力: `QPData`
- 補助出力: `extract_control()`, `extract_trajectory()`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct MPCConfig`
ホライズン、状態重み、入力重み、終端重み、入力制約、レート制約、速度制約を保持します。

### `struct QPData`
`P/q/A/l/u` と問題サイズを保持する OSQP 入力構造体です。

### `class MPCProblem`
- `NX`, `NU`: 状態次元 6、入力次元 2
- `MPCProblem(const MPCConfig&)`: 設定保持
- `formulate(...)`: コスト行列、動力学制約、入力制約、舵角レート制約、速度制約を構築
- `extract_control(...)`: 先頭入力 `(delta, ax)` を抽出
- `extract_trajectory(...)`: 解から `BicycleState6DOF` 列を復元
- `n_vars()`: 総変数数
- `config()`: 設定参照
- `var_x_offset(int)`, `var_u_offset(int)`: 内部レイアウト補助
