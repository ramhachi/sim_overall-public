# sensor_interface.hpp

## ファイルの目的
- センサモデル共通インターフェースと共有ユーティリティを定義します。

## 実行上の役割
- IMU、GPS、将来の車輪速やストロークセンサが同じ抽象基底クラスで扱えるようにし、共通の測定束と乱数器も提供します。

## 依存関係
- 外部: `Eigen/Dense`, `<array>`, `<optional>`, `<random>`, `<cmath>`
- リポジトリ内依存: `gps_model.hpp`, `imu_model.hpp`, `estimator_interface.hpp` などが利用します

## 入力/出力
- 入力: 乱数シード、平均・分散指定、センサ別値
- 出力: `SensorMeasurement`、乱数サンプル

## 関数・メソッド・クラスの網羅的な説明
- `struct SensorMeasurement`
  - `timestamp` と、各種 `std::optional` センサ値をまとめます。
  - `accel`, `gyro`, `gps_position`, `gps_velocity`, `wheel_speeds`, `steering_angle`, `suspension_travel` を持ちます。
- `class NoiseGenerator`
  - `explicit NoiseGenerator(uint64_t seed = 42)`
    - `mt19937_64` と正規/一様分布器を初期化します。
  - `gaussian(double mean = 0.0, double std = 1.0)`
    - 正規乱数を返します。
  - `uniform(double min = 0.0, double max = 1.0)`
    - 一様乱数を返します。
  - `gaussian_vector(double std)`
    - 同一標準偏差の 3 次元独立ガウスを返します。
  - `gaussian_vector(const Eigen::Matrix3d& covariance)`
    - Cholesky 分解で共分散付き 3 次元ガウスを返します。
  - `set_seed(uint64_t seed)`
    - 生成器を再初期化します。
  - private `gen_`, `normal_`, `uniform_`
    - 乱数生成本体です。
- `class ISensorModel`
  - `virtual ~ISensorModel() = default`
    - 抽象基底の仮想デストラクタです。
  - `virtual void update(double dt) = 0`
    - 真値更新後に呼ばれるセンサ更新 API です。
  - `virtual const char* name() const = 0`
    - センサ名取得 API です。
  - `virtual double sample_rate() const = 0`
    - サンプリング周波数取得 API です。
