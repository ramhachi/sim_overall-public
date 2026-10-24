# main_mpc_comparison.cpp

- **目的**: PID と 6DOF MPC の経路追従性能を同条件で比較します。
- **主要要素**: `SimResult`、`ComparisonConfig`、`run_with_pid()`、`run_with_mpc()` 系処理、`main()`。
- **理論背景**: 横偏差 RMS、速度誤差、制御レート、求解時間を同一トラック上で評価します。
- **依存**: `pid_driver.hpp`、`mpc_driver.hpp`、`vehicle_14dof_new.hpp`、`velocity_profile.hpp`。OSQP 有効時のみ MPC 経路を使います。
- **システム上の位置づけ**: Phase 8b の性能比較・回帰確認用です。

