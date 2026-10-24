# `matlab/vehicle_8dof_full/`

8DOF モデルへ Bouc-Wen、LuGre、ARB を統合した高詳細サスペンションモデル群。  
非線形ダンパとプレ・スライディング摩擦を同時に扱う。

## 収録ファイル

- `vehicle_8dof_full_model.m.md` — 8DOF + Bouc-Wen + LuGre + ARB モデル本体
- `run_8dof_full_verification.m.md` — C++ 比較および可視化

## このディレクトリの役割

14DOF 完全統合の前段として、サスペンション詳細モデルを集中検証する役目を持つ。
