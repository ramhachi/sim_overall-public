# gps_model.hpp

## ファイルの目的
- Gauss-Markov 誤差を持つ GPS センサモデルを提供します。

## 実行上の役割
- 真値位置・速度・方位から、相関位置誤差、DOP 変動、遅延、ドロップアウトを含む疑似 GPS 出力を生成します。

## 依存関係
- リポジトリ内: `sensor_interface.hpp`
- 外部: `Eigen/Dense`, `<deque>`

## 入力/出力
- 入力: 真値位置・高度・速度・方位、更新周期 `dt`
- 出力: `GPSSensorOutput`

## 関数・メソッド・クラスの網羅的な説明
- `struct GPSSensorParams`
  - 精度設定 `position_noise_std`, `altitude_noise_std`, `velocity_noise_std`, `heading_noise_std`
  - 相関時間 `correlation_time`
  - DOP 設定 `hdop_nominal`, `hdop_variation`, `vdop_nominal`
  - サンプリング `update_rate_hz`, `latency_s`
  - 信頼性 `dropout_probability`, `multipath_std`
  - を持ちます。
  - `static rtk_gps()`, `static standard_gps()`, `static fsae_typical()`
    - 用途別プリセットです。
- `struct GPSSensorOutput`
  - `position`, `altitude`, `velocity`, `heading`, `hdop`, `vdop`, `num_satellites`, `timestamp`, `valid` を保持します。
- `class GPSSensorModel : public ISensorModel`
  - `explicit GPSSensorModel(const GPSSensorParams& params = GPSSensorParams())`
    - パラメータ、乱数器、相関誤差状態を初期化します。
  - `set_ground_truth(...)`
    - 真値位置・高度・速度・方位を更新します。
  - `update(double dt) override`
    - サンプル周期に達したときのみ新しい GPS 出力を作ります。
    - ドロップアウト判定、相関誤差更新、DOP 変動、ノイズ付き位置/高度/速度/方位計算、衛星数生成、遅延付きタイムスタンプ付与を行います。
  - `output() const`
    - 最新出力を返します。
  - `params() const`
    - 設定参照を返します。
  - `name() const override`
    - `"GPS"` を返します。
  - `sample_rate() const override`
    - 更新レートを返します。
  - `set_seed(uint64_t seed)`
    - `NoiseGenerator` のシードを設定します。
  - private `update_correlated_error(double dt)`
    - 位置誤差状態 `correlated_error_` を 1 次 Gauss-Markov 過程で更新します。
  - private `compute_noisy_position(...)`
    - 相関誤差とマルチパス誤差を HDOP でスケールして足し込みます。
  - private `compute_noisy_altitude(double true_alt)`
    - 高度雑音を加えます。
  - private `compute_noisy_velocity(...)`
    - 水平速度に白色雑音を加えます。
  - private `compute_noisy_heading(double true_hdg)`
    - ヘディングに白色雑音を加えます。
  - private メンバ
    - `params_`, `noise_`, 真値群、`correlated_error_`, `output_`, `time_since_last_sample_`, `total_time_` を保持します。
