# main_14dof.cpp

- **目的**: 14DOF 車両モデルで step steer、定常円、加速、旋回加速を切り替えて実行するデモです。
- **主要要素**: `SimData`、`run_step_steer_test()`、`run_steady_state_circle_test()`、`run_acceleration_test()`、`run_cornering_acceleration_test()`、`main()`。
- **理論背景**: 1ms RK4 積分、エンジントルクマップ、Salisbury LSD、横加速度/ヨーレート評価を扱います。
- **依存**: `vehicle_14dof_new.hpp`、`spdlog`、`chrono`、`fstream`。車両本体と CSV/ログ出力に集中しています。
- **システム上の位置づけ**: フル車両モデルの代表的なベンチマーク実行ファイルです。

