# `matlab/vehicle_3dof/`

3 自由度平面車両モデルのディレクトリ。  
最小構成の車体平面運動と TMeasy タイヤを用い、C++ 実装との基礎比較を行う。

## 収録ファイル

- `vehicle_3dof_model.m.md` — 3DOF モデル本体
- `run_3dof_verification.m.md` — C++ 比較検証スクリプト

## このディレクトリの役割

6DOF / 8DOF / 14DOF へ拡張する前段として、  
横・縦・ヨーの基礎ダイナミクスとタイヤモデルの整合性を確認する。
