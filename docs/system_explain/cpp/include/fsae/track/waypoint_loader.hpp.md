# waypoint_loader.hpp

## ファイルの目的
- ウェイポイント列や CSV から `CurvatureBasedTrack` を構築します。

## 実行上の役割
- 実測・設計ウェイポイントを中心線モデルへ変換し、曲率とヘディングを近傍点から推定します。
- 近接点除去、閉ループ補完、曲率平滑化まで含む前処理器です。

## 依存関係
- リポジトリ内: `track_model.hpp`
- 外部: `<vector>`, `<fstream>`, `<sstream>`, `<string>`, `<cmath>`

## 入力/出力
- 入力: `std::vector<Waypoint>` または CSV `x,y[,width]`
- 出力: `CurvatureBasedTrack`、`std::vector<Waypoint>`

## 関数・メソッド・クラスの網羅的な説明
- `struct Waypoint`
  - `x`, `y`, `width` を持つ入力点です。
- `class WaypointTrackBuilder`
  - `WaypointTrackBuilder(std::vector<Waypoint> waypoints, double default_width = 3.0)`
    - ウェイポイント列と既定幅を保持します。
  - `build() const`
    - 2 点未満なら例外です。
    - 先頭と末尾が一致していれば `force_loop` を有効にします。
    - 各点について距離累積 `s`、左右幅、ヘディング、曲率を推定し `TrackPoint` を作ります。
    - 開ループ端点では片側差分、閉ループでは巻き戻し参照を使います。
    - 最後に移動平均で `kappa` を平滑化し、`CurvatureBasedTrack(track_points, force_loop)` を返します。
  - `static load_from_csv(const std::string& filename, double default_width = 3.0)`
    - ヘッダ有無を判定しつつ CSV を読みます。
    - `x,y` は必須、3 列目があれば幅に使います。
    - 0.5m 未満の近接ウェイポイントを除去し、終点が始点に 5m 以内ならループ閉鎖用に先頭点を追加します。
  - private `normalize_angle(double angle)`
    - `[-pi,pi]` へ繰り返し補正します。
  - private メンバ `waypoints_`, `default_width_`
    - 入力列と既定幅です。
