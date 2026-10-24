# imu_model.hpp

## ファイルの目的
- Allan 分散由来の IMU ノイズモデルを提供します。

## 実行上の役割
- 加速度計・ジャイロの白色雑音、バイアス不安定性、バイアスランダムウォーク、レンジ飽和を再現します。

## 依存関係
- リポジトリ内: `sensor_interface.hpp`
- 外部: `Eigen/Dense`

## 入力/出力
- 入力: 真値加速度・真値角速度・更新周期 `dt`
- 出力: `IMUSensorOutput`

## 関数・メソッド・クラスの網羅的な説明
- `struct IMUSensorParams`
  - 加速度計設定 `accel_noise_density`, `accel_bias_instability`, `accel_random_walk`
  - ジャイロ設定 `gyro_noise_density`, `gyro_bias_instability`, `gyro_random_walk`
  - `cross_axis_sensitivity`, `sample_rate_hz`, `latency_s`, `accel_range`, `gyro_range`
  - を持ちます。
  - `cross_axis_sensitivity` と `latency_s` はこのヘッダ内ではまだ未使用です。
  - `static tactical_grade()`, `static consumer_grade()`, `static fsae_typical()`
    - 精度別プリセットです。
- `struct IMUSensorOutput`
  - `accel`, `gyro`, `accel_bias`, `gyro_bias`, `timestamp`, `valid` を保持します。
- `class IMUSensorModel : public ISensorModel`
  - `explicit IMUSensorModel(const IMUSensorParams& params = IMUSensorParams())`
    - パラメータ、乱数器、バイアス状態を初期化します。
  - `set_ground_truth(const Eigen::Vector3d& true_accel, const Eigen::Vector3d& true_gyro)`
    - 真値を設定します。
  - `update(double dt) override`
    - サンプル周期ごとに `update_biases()` を実行し、ノイズ付き加速度・角速度を生成します。
    - 出力時刻は `total_time_` を使います。
  - `output() const`
    - 最新 IMU 出力を返します。
  - `params() const`
    - 現在設定を返します。
  - `name() const override`
    - `"IMU"` を返します。
  - `sample_rate() const override`
    - サンプリング周波数を返します。
  - `reset_bias()`
    - 現在のバイアス状態をゼロ化します。
  - `set_seed(uint64_t seed)`
    - 乱数器シードを更新します。
  - private `update_biases(double dt)`
    - 一次マルコフ減衰付きランダムウォークで各軸バイアスを更新し、`±3σ` にクランプします。
  - private `compute_noisy_accel(const Eigen::Vector3d& true_accel)`
    - 真値 + バイアス + 白色雑音を作り、計測レンジにクランプします。
  - private `compute_noisy_gyro(const Eigen::Vector3d& true_gyro)`
    - ジャイロ版です。
  - private メンバ
    - `params_`, `noise_`, 真値群、`accel_bias_`, `gyro_bias_`, `output_`, `time_since_last_sample_`, `total_time_` を保持します。
