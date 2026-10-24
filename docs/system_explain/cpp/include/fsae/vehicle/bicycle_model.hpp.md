# bicycle_model.hpp

## 1. ファイルの目的
MPC 用の 6 自由度動的自転車モデルを定義するヘッダです。

## 2. 実行上の役割
状態微分、RK4 予測、有限差分線形化、定常旋回解析を提供し、MPC の予測モデルになります。

## 3. 依存関係
- `vehicle_params.hpp`
- `control/mpc_problem.hpp`, `driver/mpc_driver.hpp`

## 4. 入力/出力
- 入力: `BicycleState6DOF`, `BicycleControl`, `dt`
- 出力: 状態微分、次状態、線形化行列、定常旋回解析値

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct BicycleState6DOF`
`x,y,psi,vx,vy,r` を持ち、`to_vector()`, `from_vector(...)` を提供します。

### `struct BicycleControl`
`delta`, `ax` を持ち、`to_vector()`, `from_vector(...)` を提供します。

### `struct BicycleParams`
`m`, `Izz`, `lf`, `lr`, `Cf`, `Cr` を持ち、`from_vehicle_params(...)` で 14DOF 設定から簡略値を抽出します。

### `class BicycleModel6DOF`
- `BicycleModel6DOF(const BicycleParams&)`
- `compute_derivative(...)`
- `predict_step(...)`
- `linearize(...)`
- `steady_state_cornering(...)`
- `params()`
