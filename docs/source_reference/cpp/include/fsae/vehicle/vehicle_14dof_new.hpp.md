# vehicle_14dof_new.hpp

- **目的**: 現行の 14DOF フル車両モデルを統合実装します。
- **主要要素**: `State14DOF`、`Vehicle14DOF_New`、適応タイムステップ、POU、フレームねじり、ステアリングトルク、DRS、複数タイヤモデル切替。
- **理論背景**: TMeasy/Magic Formula/NN タイヤ、Bouc-Wen、LuGre、LSD、4D エアロ、熱モデルを 44 状態で統合します。
- **依存**: `vehicle_params.hpp`、`vehicle_state.hpp`、`drs_system.hpp`、`steering_torque.hpp`、各 physics ヘッダ、`Eigen`。
- **システム上の位置づけ**: C++ シミュレーション全体の中核です。

