# `cpp/src/test_track_model.cpp`

## 1. ファイルの目的
トラック生成、Frenet 座標変換、レーシングライン計算、ウェイポイント読み込みを一通り確認するトラックモジュール総合デモです。

## 2. 実行上の役割
- `track` 名前空間の主要 API をまとめて試すサンプルです。
- 生成結果を CSV に残すため、可視化や外部確認に使えます。
- 経路追従やラップタイム側から利用する前の基礎確認にも向いています。

## 3. 依存関係
- `fsae/track/track_model.hpp`
- `fsae/track/frenet.hpp`
- `fsae/track/waypoint_loader.hpp`
- `create_circle_track()`, `create_figure_eight_track()`, `create_straight_corner_track()`
- `compute_racing_line()`
- `FrenetConverter`
- `Waypoint`, `WaypointTrackBuilder`, `WaypointTrackBuilder::load_from_csv()`
- 標準ライブラリ `iostream`, `fstream`, `iomanip`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 円トラック: 半径 30 m, 幅 3 m
  - 8 の字: 半径 25 m
  - 直線+コーナー: 直線 50 m, 半径 15 m
  - Frenet 変換テスト点: `(35,0,π/2)`
  - ウェイポイント列はファイル内で定義
- 出力:
  - `circle_track.csv`
  - `figure_eight_track.csv`
  - `straight_corner_track.csv`
  - `racing_line.csv`
  - `waypoint_track.csv`
  - `test_waypoints.csv`
  - 標準出力にトラック長、Frenet 変換結果、CSV ロード結果
  - 明示的 pass/fail 判定なし

## 5. 関数・メソッド・クラスの網羅的な説明

### `test_circle_track()`
- `create_circle_track(30.0, 3.0)` を生成します。
- `track.length()` を表示します。
- `track.points()` の `s,x,y,theta,kappa` を `circle_track.csv` に保存します。
- 確認内容:
  - 円トラック生成結果の妥当性

### `test_figure_eight_track()`
- `create_figure_eight_track(25.0, 3.0)` を生成します。
- 点列を `figure_eight_track.csv` に保存します。
- 確認内容:
  - 8 の字トラック生成と曲率変化

### `test_straight_corner_track()`
- `create_straight_corner_track(50.0, 15.0, 3.0)` を生成します。
- 点列を `straight_corner_track.csv` に保存します。
- 確認内容:
  - 直線から定曲率コーナーへの接続

### `test_frenet_conversion()`
- 円トラックから `FrenetConverter` を生成します。
- テスト点 `(35,0,π/2)` を `global_to_frenet()` で Frenet 化します。
- その結果を `frenet_to_global()` で逆変換します。
- `is_within_track()` でコース内判定も行います。
- 確認内容:
  - Frenet 変換と逆変換の整合性

### `test_racing_line()`
- 幅 4 m の 8 の字トラックで `compute_racing_line(track, 0.5)` を実行します。
- `FrenetConverter` でグローバル座標へ戻し、`racing_line.csv` に保存します。
- 確認内容:
  - レーシングライン計算と座標変換の連携

### `test_waypoint_loader()`
- 手書き `std::vector<Waypoint>` から `WaypointTrackBuilder` を構築します。
- `build()` でトラックを生成し、`waypoint_track.csv` に保存します。
- 同じウェイポイントを `test_waypoints.csv` として書き出します。
- `WaypointTrackBuilder::load_from_csv("test_waypoints.csv")` で再読込します。
- 確認内容:
  - 手書きウェイポイント生成
  - CSV ロード API の動作

### `main()`
- このファイルの入口です。
- 以下を順に実行します。
  1. `test_circle_track()`
  2. `test_figure_eight_track()`
  3. `test_straight_corner_track()`
  4. `test_frenet_conversion()`
  5. `test_racing_line()`
  6. `test_waypoint_loader()`

### このファイル内で新規定義される型・補助関数
- 関数:
  - `test_circle_track`
  - `test_figure_eight_track`
  - `test_straight_corner_track`
  - `test_frenet_conversion`
  - `test_racing_line`
  - `test_waypoint_loader`
  - `main`
- 独自 `class` / `struct`: なし
