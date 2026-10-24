# lsd.hpp

## ファイルの目的
- Salisbury 式 LSD の差動制限トルクと左右輪トルク配分を表現します。

## 実行上の役割
- 入力トルクと左右輪速度差からロックトルクを求め、オープンデフより現実的な左右配分を与えます。
- 加速側と減速側で異なるランプ角を使えるため、進入安定性と脱出トラクションの差を再現できます。

## 依存関係
- 外部: `<cmath>`, `<algorithm>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `input_torque`, `omega_left`, `omega_right`, `total_torque`
- 出力: ロックトルク、左右輪トルク、バイアス比

## 関数・メソッド・クラスの網羅的な説明
- `class SalisburyLSD`
  - `struct Params`
    - `ramp_angle_accel`, `ramp_angle_decel`, `preload_torque`, `effective_radius`, `clutch_friction`, `num_plates` を保持します。
    - `static fsae_traction_focused()`
      - 加速側ロックを強めた既定セットです。
    - `static fsae_balanced()`
      - 中庸な既定セットです。
    - `static fsae_handling_focused()`
      - 回頭性重視でロックを弱めた既定セットです。
  - `explicit SalisburyLSD(const Params& params = Params::fsae_traction_focused())`
    - パラメータを保持して初期化します。
  - `set_params(const Params& params)`
    - 現在の LSD 設定を差し替えます。
  - `get_params() const`
    - 保持中の設定を参照返しします。
  - `compute_locking_torque(double input_torque, double omega_left, double omega_right) const`
    - 入力トルクの符号で加減速を判定し、対応するランプ角を選びます。
    - `T_pre + |T_input|/tan(theta) * R_eff * mu * Nplates` で制限トルクを返します。
    - 現在の式では左右速度差自体は使わず、加減速条件だけが効きます。
  - `distribute_torque(double total_torque, double omega_left, double omega_right, double& torque_left, double& torque_right) const`
    - ベースは均等配分です。
    - 速度差がほぼ 0 なら完全ロック相当として均等配分のまま返します。
    - 速度差がある場合は `locking_torque` と `base_torque` の小さい方をバイアス量とし、速い輪から遅い輪へトルクを移します。
  - `compute_bias_ratio(double input_torque, double omega_left, double omega_right) const`
    - `distribute_torque()` の結果から速い側/遅い側トルク比を返します。
    - 低速側トルクがほぼ 0 なら 1.0 を返します。
  - private `params_`
    - LSD 設定本体です。
