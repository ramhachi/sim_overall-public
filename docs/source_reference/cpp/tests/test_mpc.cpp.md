# test_mpc.cpp

- **目的**: TrackDistanceQuery、MPC QP 生成、sampling preview MPC ドライバーの制御出力と計算時間を検証します。
- **主要要素**: `TrackDistanceTest`、`MPCProblemTest`、`MPCDriverTest`。
- **理論背景**: 最近接点探索、QP 次元、単純トラック上の有限ホライズン予測制御を確認します。
- **依存**: `gtest`、`track_distance.hpp`、`mpc_problem.hpp`、`mpc_driver.hpp`。
- **システム上の位置づけ**: 6DOF MPC 周辺の統合回帰です。
