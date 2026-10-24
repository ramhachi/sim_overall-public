# `cpp/src/test_velocity_profile.cpp`

## 1. ファイルの目的
ラップタイム用速度プロファイル生成器が、各種トラックに対して妥当な速度・縦加速度・横加速度制約を反映したプロファイルを作れるかを確認するテストプログラムです。

## 2. 実行上の役割
- `laptime` モジュールの代表 API をまとめて試すサンプルです。
- トラックモデルとの接続確認、g-g ダイアグラム生成、性能差比較まで含みます。
- ラップタイム解析前処理の回帰確認にも使えます。

## 3. 依存関係
- `fsae/laptime/velocity_profile.hpp`
- `fsae/track/track_model.hpp`
- `VehiclePerformance`, `VelocityProfileGenerator`
- `compute_lap_time()`, `generate_gg_diagram()`
- `create_circle_track()`, `create_figure_eight_track()`, `create_straight_corner_track()`
- 標準ライブラリ `iostream`, `fstream`, `iomanip`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 標準性能として `max_lat_accel=1.5G`, `max_lon_accel=1.0G`, `max_lon_decel=1.8G`
  - 必要に応じて `max_speed`, `min_speed` を設定
  - 対象:
    - 円トラック
    - 8 の字トラック
    - 直線+コーナー
    - g-g ダイアグラム
    - 標準車 vs エアロ強化車
- 出力:
  - `circle_velocity_profile.csv`
  - `figure_eight_velocity_profile.csv`
  - `straight_corner_velocity_profile.csv`
  - `gg_diagram.csv`
  - 標準出力にラップタイム、区間数、性能比較結果
  - 明示的 pass/fail 判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `test_circle_track_velocity_profile()`
- 円トラック `create_circle_track(30.0, 3.0)` を使います。
- `VelocityProfileGenerator` で 1 m 間隔の速度プロファイルを生成します。
- `compute_lap_time()` でラップタイムを計算します。
- `s,v,ax,ay,v_kmh` を `circle_velocity_profile.csv` に保存します。
- 確認内容:
  - 定曲率コースでの基礎速度プロファイル生成

### `test_figure_eight_velocity_profile()`
- `create_figure_eight_track(25.0, 3.0)` を使います。
- 1 m 間隔の速度プロファイルを生成し、ラップタイムを計算します。
- `figure_eight_velocity_profile.csv` に保存します。
- 確認内容:
  - 曲率が変化するコースでの速度制約適用

### `test_straight_corner_velocity_profile()`
- `create_straight_corner_track(50.0, 15.0, 3.0)` を使います。
- `generate(track, 0.5)` で 0.5 m 間隔のプロファイルを作ります。
- `ax` を見て
  - 加速区間
  - 制動区間
  - 惰行区間
  を数えます。
- `straight_corner_velocity_profile.csv` に保存します。
- 確認内容:
  - 加減速区間の発生位置

### `test_gg_diagram()`
- `generate_gg_diagram(perf, 100)` を呼びます。
- `(ax, ay)` 点群を `gg_diagram.csv` に保存します。
- `ax_g`, `ay_g` として G 値も併記します。
- 確認内容:
  - g-g ダイアグラム生成 API の動作

### `test_performance_comparison()`
- 円トラックで、標準性能車 `perf_std` とエアロ強化車 `perf_aero` を比較します。
- 両者で速度プロファイルを作り、`compute_lap_time()` を比較します。
- 秒差と改善率を表示します。
- 確認内容:
  - 性能パラメータ変更がラップタイムへ与える影響

### `main()`
- このファイルの入口です。
- 以下を順に実行します。
  1. `test_circle_track_velocity_profile()`
  2. `test_figure_eight_velocity_profile()`
  3. `test_straight_corner_velocity_profile()`
  4. `test_gg_diagram()`
  5. `test_performance_comparison()`

### このファイル内で新規定義される型・補助関数
- 関数:
  - `test_circle_track_velocity_profile`
  - `test_figure_eight_velocity_profile`
  - `test_straight_corner_velocity_profile`
  - `test_gg_diagram`
  - `test_performance_comparison`
  - `main`
- 独自 `class` / `struct`: なし
