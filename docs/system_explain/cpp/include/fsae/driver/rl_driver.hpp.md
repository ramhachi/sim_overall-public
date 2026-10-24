# rl_driver.hpp

## 1. ファイルの目的
学習済み RL Actor ネットワークを C++ で推論するドライバーを定義するヘッダです。

## 2. 実行上の役割
観測構築、正規化、MLP forward、行動デコード、安全速度制限までを行い、学習済みポリシーを `DriverInput` に変換します。

## 3. 依存関係
- `vehicle/vehicle_14dof_new.hpp`, `vehicle/vehicle_params.hpp`, `vehicle/vehicle_state.hpp`
- `track/track_model.hpp`, `track/frenet.hpp`

## 4. 入力/出力
- 入力: `State14DOF`, トラック, `FrenetConverter`, 学習済み重みディレクトリ
- 出力: `DriverInput`

## 5. 関数・メソッド・クラスの網羅的な説明
### `rl_driver_speed_limits_disabled()`
曲率ベース安全速度制限を無効化するか、環境変数から判定します。

### `struct RLFCLayer`
層名、重み、バイアス、LayerNorm 係数を保持します。`forward(...)` で線形変換と必要なら LayerNorm を実行します。

### `class RLDriver`
- 定数: `SERVER_OBS_DIM`, `N_VEHICLE_PARAMS`, `OBS_DIM`
- `RLDriver()`
- `set_vehicle_params(...)`
- `load(...)`
- `is_loaded()`, `act_dim()`, `obs_dim()`
- `infer_normalized_action(...)`
- `compute_control(...)`: 観測生成→正規化→推論→行動デコード→安全制限
- `build_observation(...)`
- `select_gear(double)`
- `apply_curvature_speed_limit(...)`
- `load_weights(...)`
- `load_normalization(...)`
- `validate_steering_scale_contract()`
