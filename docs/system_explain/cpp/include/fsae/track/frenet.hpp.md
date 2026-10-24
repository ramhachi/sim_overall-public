# frenet.hpp

## ファイルの目的
- トラック中心線基準の Frenet 座標変換を提供します。

## 実行上の役割
- 車両の全体座標 `(x,y,psi)` を `(s,n,d_heading)` に変換し、逆変換やトラック内判定も行います。
- 自己交差トラックや閉ループでの `s` 連続性を保つため、前回 `s` を使った探索制御を持ちます。

## 依存関係
- リポジトリ内: `track_model.hpp`
- 外部: `<cmath>`, `<limits>`, `<vector>`, `<tuple>`, `<algorithm>`

## 入力/出力
- 入力: グローバル座標・ヨー角、`FrenetCoordinate`
- 出力: `FrenetCoordinate`、`(x,y,theta)` タプル、判定真偽

## 関数・メソッド・クラスの網羅的な説明
- `struct FrenetCoordinate`
  - `s`, `n`, `d_heading` を保持します。
- `class FrenetConverter`
  - `explicit FrenetConverter(const CurvatureBasedTrack& track)`
    - 対象トラックを参照保持します。
  - `global_to_frenet(double x, double y, double psi) const`
    - `track_.points()` 上の候補を探索し、距離・ヘディング誤差・後退ジャンプ罰則を組み合わせたコストで最近点を選びます。
    - 閉ループなら `unwrap_s()` / `wrap_s()` で周回をまたぐ `s` の連続性を保ちます。
    - 結果から法線方向投影で横偏差 `n` を求め、`d_heading` を正規化して返します。
  - `frenet_to_global(const FrenetCoordinate& frenet) const`
    - `track_.get_point(s)` を基準に法線方向へ `n` だけオフセットし、姿勢は `theta + d_heading` で返します。
  - `is_within_track(const FrenetCoordinate& frenet) const`
    - その `s` での左右幅の小さい方に `|n|` が収まるか判定します。
  - `track() const`
    - 内部トラック参照を返します。
  - `reset()`
    - 連続探索用の `prev_s_` 状態を初期化します。
  - private `normalize_angle(double angle)`
    - `[-pi,pi)` へ正規化し、NaN/Inf は 0 にします。
  - private メンバ `track_`, `initialized_`, `prev_s_`, `prev_s_unwrapped_`
    - 変換対象と連続探索状態です。
  - private 定数 `search_window_`, `fallback_threshold_`, `max_progress_jump_`, `heading_weight_`, `heading_gate_`, `backward_slack_`, `backward_penalty_weight_`
    - 候補探索範囲とコスト係数です。
- `compute_racing_line(const CurvatureBasedTrack& track, double safety_margin = 0.5)`
  - 各トラック点で曲率に応じて外側へ寄る単純なレーシングラインを作ります。
  - `|kappa|` が大きいほど `max_offset` まで寄せ、向き偏差 `d_heading` は 0 に固定されます。
