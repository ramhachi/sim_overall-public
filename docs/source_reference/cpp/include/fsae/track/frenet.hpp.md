# frenet.hpp

- **目的**: グローバル座標と Frenet 座標 `(s,n,Δheading)` を相互変換します。
- **主要要素**: `FrenetCoordinate`、`FrenetConverter`、`compute_racing_line()`。
- **理論背景**: 最近接中心線点を使って横偏差と進行方向誤差を計算し、自己交差コースにも配慮します。
- **依存**: `track_model.hpp`、`tuple`、`limits`。
- **システム上の位置づけ**: PID ドライバーやレーシングライン計算の座標変換層です。

