# `matlab/vehicle_14dof/`

14 自由度完全車両モデルと、その補助サブシステム・検証スクリプト群。  
空力、タイヤ、ダンパ、摩擦、LSD、エンジン、熱モデル、数値積分比較まで含む MATLAB 側フル統合領域である。

## 収録ファイル

- `aero_map_4d.m.md`
- `engine_torque_map.m.md`
- `nn_tire_predict.m.md`
- `run_14dof_cpp_comparison.m.md`
- `run_14dof_nn_simulation.m.md`
- `run_14dof_verification.m.md`
- `run_rosenbrock_verification.m.md`
- `run_stiff_model_comparison.m.md`
- `salisbury_lsd.m.md`
- `tire_thermal_update.m.md`
- `vehicle_14dof_model.m.md`
- `vehicle_14dof_nn.m.md`
- `verify_params_match.m.md`

## このディレクトリの役割

MATLAB における完全車両ダイナミクスの参照実装・検証基盤として、  
フルビークル挙動、サブシステム単体確認、C++ 比較、NN 統合評価を一括で支える。
