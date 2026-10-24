# pid_driver.hpp

## 1. ファイルの目的
PID ベースの経路追従ドライバーを定義するヘッダです。

## 2. 実行上の役割
Frenet 座標系の横偏差・ヘディング誤差・速度誤差から操舵と加減速を計算します。トルクモード時は内側ステアリング PID も使います。

## 3. 依存関係
- `vehicle/vehicle_14dof_new.hpp`
- `track/track_model.hpp`, `track/frenet.hpp`
- `laptime/velocity_profile.hpp`

## 4. 入力/出力
- 入力: `State14DOF`, 時刻
- 出力: `DriverInput`

## 5. 関数・メソッド・クラスの網羅的な説明
### `driver_max_throttle_unlimited()`
環境変数 `FOPT_UNLIMIT_DRIVER_MAX_THROTTLE` から最大スロットル制限の解除有無を判定します。

### `driver_corner_throttle_limit_disabled()`
環境変数 `FOPT_DISABLE_DRIVER_CORNER_THROTTLE_LIMIT` からコーナリング時スロットル制限の無効化有無を判定します。

### `struct PIDGains`
ステアリング外側/内側ループ、速度 PI、各種上限、先読み距離などのゲイン設定です。

### `struct PIDState`
横偏差積分、速度誤差積分、前回横偏差、前回時刻を保持します。

### `class PIDPathFollowingDriver`
- `PIDPathFollowingDriver(...)`
- `compute_control(...)`: Frenet 変換、目標速度取得、操舵計算、速度制御、DRS 要求、ギア選択を実施
- `reset()`
- `set_gains(...)`
- `get_current_frenet(...)`
- `get_target_speed(double)`
- `wrap_track_s(double)`
- `get_preview_min_target_speed(...)`
- `compute_steering_control(...)`
- `compute_progressive_throttle_limit(...)`
- `compute_speed_control(...)`
- `select_gear(double)`

### プリセット関数
- `conservative_gains()`
- `aggressive_gains()`
- `optimized_figure_eight_gains()`
- `optimized_circle_gains()`
- `optimized_skidpad_gains()`
- `optimized_autocross_gains()`
