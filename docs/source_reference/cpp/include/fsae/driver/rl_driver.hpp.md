# rl_driver.hpp

- **目的**: テキスト出力された RL Actor を C++ 側で読み込み、制御入力へ変換します。
- **主要要素**: `RLFCLayer`、`RLDriver`。
- **理論背景**: 正規化済み観測から多層全結合ネットを前向き計算し、2D または 3D 行動空間を物理入力へ写像します。
- **依存**: `vehicle_14dof_new.hpp`、`track_model.hpp`、`frenet.hpp`、`fstream`。
- **システム上の位置づけ**: 学習済みポリシーをシミュレータへ接続する推論層です。

