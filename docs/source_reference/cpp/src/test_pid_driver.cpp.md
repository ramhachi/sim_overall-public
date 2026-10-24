# test_pid_driver.cpp

- **目的**: PID 経路追従ドライバーで円・8 の字などを追従させ、誤差を CSV 出力します。
- **主要要素**: `test_pid_circle_tracking()`、`test_pid_figure_eight_tracking()`、`main()`。
- **理論背景**: Frenet 横偏差・ヘディング誤差を使う PID + 速度プロファイル追従の手動評価です。
- **依存**: `pid_driver.hpp`、`vehicle_14dof_new.hpp`、`track_model.hpp`、`velocity_profile.hpp`。
- **システム上の位置づけ**: Level 1 ドライバーの挙動検証用です。

