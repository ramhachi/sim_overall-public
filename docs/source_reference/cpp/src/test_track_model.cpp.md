# test_track_model.cpp

- **目的**: 円、8 の字、複合トラック、Frenet 変換、レーシングライン生成をまとめて検証します。
- **主要要素**: `test_circle_track()`、`test_figure_eight_track()`、`test_straight_corner_track()`、`test_frenet_conversion()`、`test_racing_line()`、`main()`。
- **理論背景**: 曲率ベース中心線、Frenet 座標、最近接点変換の妥当性を CSV と標準出力で確認します。
- **依存**: `track_model.hpp`、`frenet.hpp`、`waypoint_loader.hpp`。
- **システム上の位置づけ**: 経路追従やラップタイムの前提となるトラック表現の手動検証です。

