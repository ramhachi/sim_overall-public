# main_mpc_tuning.cpp

- **目的**: MPC の重みと等価コーナリング剛性候補をグリッドサーチし、良い設定を探索します。
- **主要要素**: `TuningResult`、`run_mpc_tuning()`、`main()`。
- **理論背景**: 円形トラック上で横偏差、舵角レート、ラップタイム、OSQP 求解時間を評価し、最良設定を書き出します。
- **依存**: `pid_driver.hpp`、`mpc_driver.hpp`、`vehicle_14dof_new.hpp`、`velocity_profile.hpp`、`algorithm`。
- **システム上の位置づけ**: Phase 8c のチューニング支援ツールです。

