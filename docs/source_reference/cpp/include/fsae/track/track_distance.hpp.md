# track_distance.hpp

- **目的**: トラック中心線への最近接点検索と MPC 用参照軌道生成を高速化します。
- **主要要素**: `NearestResult`、`ReferencePoint`、`TrackDistanceQuery`。
- **理論背景**: 等間隔サンプルと `hint_s` による局所探索で O(1) に近い最近接探索を狙います。
- **依存**: `track_model.hpp`、`vector`、`limits`。
- **システム上の位置づけ**: MPC 系が共有する最近接/先読み軌道インフラです。

