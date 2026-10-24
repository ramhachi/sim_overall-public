# engine.hpp

## ファイルの目的
- エンジントルクカーブとギア比計算の簡易パワートレイン部品を提供します。

## 実行上の役割
- 回転数から駆動トルクを得る処理と、エンジン回転数↔車輪角速度変換を担います。
- 実車系では CBR600RR を想定した初期値生成も提供します。

## 依存関係
- 外部: `<vector>`, `<algorithm>`, `<cmath>`, `<stdexcept>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: RPM 配列、トルク配列、`rpm`, `throttle`, `gear`, `wheel_omega`
- 出力: 補間トルク、ピーク情報、総減速比、回転数変換結果

## 関数・メソッド・クラスの網羅的な説明
- `class EngineTorqueMap`
  - `EngineTorqueMap()`
    - 空のトルクマップを作ります。
  - `set_curve(const std::vector<double>& rpm_points, const std::vector<double>& torque_points)`
    - RPM とトルクの対応表を設定します。
    - サイズ不一致、2 点未満、RPM 非昇順なら `std::invalid_argument` を送出します。
  - `get_torque(double rpm, double throttle = 1.0) const`
    - RPM を範囲内にクランプし、`interpolate()` で最大トルクを線形補間後、スロットル開度で比例縮小します。
    - 曲線未設定なら 0 を返します。
  - `get_peak_torque_rpm() const`
    - 最大トルク値が現れる RPM を返します。
  - `get_peak_torque() const`
    - 最大トルク値そのものを返します。
  - `get_idle_rpm() const`
    - 最初の RPM 点、未設定時は 1000 rpm を返します。
  - `get_redline_rpm() const`
    - 最後の RPM 点、未設定時は 14500 rpm を返します。
  - private `interpolate(double rpm) const`
    - `std::lower_bound` で区間を見つけ、線形補間します。
- `inline EngineTorqueMap create_cbr600rr_engine()`
  - CBR600RR 想定の RPM/トルク表を埋めた `EngineTorqueMap` を返します。
- `struct GearRatios`
  - `ratios`, `primary_ratio`, `final_ratio` を保持します。
  - `static cbr600rr_default()`
    - CBR600RR 標準ギア比を返します。
  - `get_total_ratio(int gear) const`
    - 指定ギアの総減速比を返します。
    - `gear<=0` や範囲外はニュートラル扱いで 0 を返します。
  - `engine_rpm_to_wheel_omega(double engine_rpm, int gear) const`
    - エンジン回転数を車輪角速度 `rad/s` に換算します。
  - `wheel_omega_to_engine_rpm(double wheel_omega, int gear) const`
    - 車輪角速度からエンジン回転数を逆算します。
