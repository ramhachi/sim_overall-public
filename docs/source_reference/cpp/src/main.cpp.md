# main.cpp

- **目的**: クォーターカー系の最小デモを実行し、ばね上/ばね下/ダンパ/タイヤ力の時系列 CSV を出力します。
- **主要要素**: `TimeSeriesData`、`run_quarter_car_demo()`、`main()`。
- **理論背景**: Bouc-Wen ダンパー付き quarter-car モデルを RK4 で 1ms 積分し、段差入力に対する応答を評価します。
- **依存**: `tmeasy.hpp`、`bouc_wen.hpp`、`rotation.hpp`、`Eigen`、`spdlog`、`nlohmann/json`。計算・ログ・将来拡張用の JSON 依存をまとめています。
- **システム上の位置づけ**: 全車両モデルへ進む前に、サスペンション/タイヤ基礎要素を単独確認する最初の実行例です。

