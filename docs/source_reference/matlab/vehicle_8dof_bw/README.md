# `matlab/vehicle_8dof_bw/`

8DOF モデルへ Bouc-Wen ダンパと ARB を追加したディレクトリ。  
サスペンションの非線形ヒステリシスを車両全体に統合し、C++ と比較できる形で検証する。

## 収録ファイル

- `vehicle_8dof_bw_model.m.md` — 8DOF + Bouc-Wen + ARB モデル本体
- `run_8dof_bw_verification.m.md` — C++ 比較および MATLAB 単体確認

## このディレクトリの役割

線形減衰から一歩進み、現実的なダンパヒステリシスを含むサスペンション解析段階を担う。
