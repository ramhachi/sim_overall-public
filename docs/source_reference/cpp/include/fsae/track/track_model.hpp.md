# track_model.hpp

- **目的**: 曲率ベースのトラック中心線モデルを提供します。
- **主要要素**: `TrackPoint`、`CurvatureBasedTrack`、トラック生成補助関数群。
- **理論背景**: 弧長 `s` と曲率 `κ(s)` から中心線座標と幅を再構成します。
- **依存**: `vector`、`algorithm`、`stdexcept`。
- **システム上の位置づけ**: 経路追従・ラップタイム・RL 観測の基盤です。

