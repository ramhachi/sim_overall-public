# fsae_rl_server.cpp

- **目的**: MATLAB RL Toolbox など外部クライアントから 14DOF シミュレータを TCP 経由で操作するサーバです。
- **主要要素**: バイナリプロトコル定数群、`RLServer`、`handle_client()`、`main()`。
- **理論背景**: 20 個の 1ms 物理ステップを 1 回の RL step にまとめ、34 次元観測と補助情報を返します。
- **依存**: `vehicle_14dof_new.hpp`、`track/*`、`velocity_profile.hpp`、`nlohmann/json`、POSIX socket API。
- **システム上の位置づけ**: 学習ループと C++ 物理コアを結ぶオンライン連携層です。

