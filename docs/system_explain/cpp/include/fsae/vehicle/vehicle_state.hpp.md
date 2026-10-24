# vehicle_state.hpp

## 1. ファイルの目的
14DOF 系シミュレーションでやり取りする共通状態・入力・出力構造体を定義するヘッダです。車両本体とドライバー系の橋渡しに使われます。

## 2. 実行上の役割
`VehicleState` はドライバーやログ出力の共通状態型、`DriverInput` は各ドライバーの出力型、`WheelState` と `SimOutput` は中間計算や結果保存用の入れ物として機能します。

## 3. 依存関係
- `vehicle_14dof_new.hpp`、各ドライバー、ログ・可視化系
- `Eigen/Dense`: ベクトル変換

## 4. 入力/出力
- 入力: 状態ベクトルやドライバー入力を表現するデータ構造
- 出力: `to_vector()/from_vector()`、`SimOutput`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct VehicleState`
- ばね上: `x,y,z,phi,theta,psi,vx,vy,vz,p,q,r`
- ばね下: `z_u[4]`, `vz_u[4]`
- 車輪回転: `omega[4]`
- タイヤ状態: `tire_xi[4]`, `tire_eta[4]`
- ダンパー状態: `damper_z[4]`
- `STATE_SIZE = 40`
- `to_vector()`: 40 次元 Eigen ベクトル化
- `from_vector(...)`: ベクトルから復元
- `speed()`: 平面速度
- `slip_angle_body()`: 車体スリップ角

### `struct DriverInput`
ドライバー入力です。`steering`, `throttle`, `brake`, `gear`, `drs_button` を持ちます。

### `struct WheelState`
各輪の中間計算用状態です。位置、ホイール座標系速度、`slip_angle`, `slip_ratio`, `Fx/Fy/Fz`, `Mz`, `susp_travel`, `susp_velocity`, `F_spring`, `F_damper`, `steer_angle` を持ちます。

### `struct SimOutput`
1 サンプル分のシミュレーション出力です。`time`, `state`, `wheels`, 派生加速度 `ax/ay/az`, 空力 `Fz_aero`, `Fx_aero` を持ちます。
