# waypoint_loader.hpp

- **目的**: ウェイポイント列や CSV から `CurvatureBasedTrack` を生成します。
- **主要要素**: `Waypoint`、`WaypointTrackBuilder`。
- **理論背景**: 点列からヘディング・曲率を近似し、必要に応じて閉ループ判定と平滑化を行います。
- **依存**: `track_model.hpp`、`fstream`、`sstream`。
- **システム上の位置づけ**: 実測/設計ウェイポイントをシミュレーション用トラックへ変換する入口です。

