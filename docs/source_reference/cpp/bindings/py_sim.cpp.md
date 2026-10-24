# py_sim.cpp

- **目的**: TMeasy、熱モデル、Bouc-Wen、LuGre、回転行列を Python へ公開します。
- **主要要素**: `PYBIND11_MODULE(fsae_sim_py, m)`。
- **理論背景**: C++ 実装を Python から直接評価し、フィッティングや可視化と接続するための薄いバインディングです。
- **依存**: `pybind11`、`tmeasy.hpp`、`thermal.hpp`、`bouc_wen.hpp`、`lugre.hpp`、`rotation.hpp`。
- **システム上の位置づけ**: Python ワークフローと C++ 物理コアの橋渡しです。

