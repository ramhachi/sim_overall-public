# suspension_kinematics_design/

サスペンション幾何最適化サブシステムの入口です。  
EasyStart のメニュー [7] から辿る **Front A-Arm -> Rear A-Arm -> Steering / POU** のうち、ここでは runtime artifact を基準にした A アーム逆解析を担います。

## 現在の主フロー

1. `main_a_arm_optimizer.m`
2. `main_rear_a_arm_optimizer.m`
3. `build_inverse_targets.m`
4. `evaluate_double_wishbone_3d.m`
5. 結果を同じ車両 artifact へ追記

## このサブシステムがしていること

- 車両 artifact からトレッド、静的キャンバー、`design_targets`（未指定時はフォールバック目標）を読む
- A アーム取付点を `fmincon + MultiStart` で探索する
- `evaluate_double_wishbone_3d.m` で 3D travel / roll / steer を評価する
- `params.suspension.kc.*` と `params.suspension_geometry.*` を同じ artifact に保存する

## ファイル

- [main_a_arm_optimizer.m.md](main_a_arm_optimizer.m.md)
- [main_rear_a_arm_optimizer.m.md](main_rear_a_arm_optimizer.m.md)
- [build_inverse_targets.m.md](build_inverse_targets.m.md)
- [evaluate_double_wishbone_3d.m.md](evaluate_double_wishbone_3d.m.md)
- [a_arm_kinematics_solver.m.md](a_arm_kinematics_solver.m.md)
- [plot_a_arm_geometry.m.md](plot_a_arm_geometry.m.md)
- [tests/README.md](tests/README.md)
