# `matlab/vehicle_6dof/`

6 自由度車両モデルのディレクトリ。  
ばね上剛体の 6DOF とタイヤ・空力・駆動を組み合わせ、姿勢運動を含む基礎モデルを検証する。

## 収録ファイル

- `vehicle_6dof_model.m.md` — 6DOF モデル本体
- `run_6dof_verification.m.md` — MATLAB / C++ 比較スクリプト

## このディレクトリの役割

平面 3DOF から、ヒーブ・ロール・ピッチを含む立体運動へ拡張する中間段階を担う。
