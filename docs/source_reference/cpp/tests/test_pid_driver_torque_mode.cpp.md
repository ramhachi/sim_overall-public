# test_pid_driver_torque_mode.cpp

- **目的**: PID ドライバーの torque mode 内側ループを検証します。
- **主要要素**: `PIDDriverTorqueTest`。
- **理論背景**: 目標舵角からドライバートルクへ変換する内側 PID が減衰と飽和を適切に扱うかを見ます。
- **依存**: `gtest`、`pid_driver.hpp`、`vehicle_14dof_new.hpp`。
- **システム上の位置づけ**: torque-mode PID の回帰です。
