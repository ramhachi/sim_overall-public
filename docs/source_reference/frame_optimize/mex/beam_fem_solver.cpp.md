# frame_optimize/mex/beam_fem_solver.cpp

## 役割

MATLAB から直接呼ぶ MEX 版 3D 梁 FEM ソルバーです。`result = beam_fem_solver(nodes, elements, bcs, loads)` という形で使えます。

## 返すもの

- `U`: 節点変位・回転
- `K_torsion`: ねじり剛性
- `mass`: 総質量
- `success`: 成功フラグ

## サーバー版との関係

実装の基本は `beam_fem_server.cpp` とよく似ていますが、現在の EasyStart 最適化経路では **こちらは標準ではありません**。通常は `beam_fem_solver_tcp_java` → TCP サーバー版が使われます。

## 実装差分として覚えておきたい点

- 浮遊節点対策は「要素に一度も現れない節点」の拘束が中心で、サーバー版のような到達可能性 BFS ではありません
- ねじり剛性の計算は直接トルク負荷中心で、Fz カップル検出は入っていません

そのため、MEX 版は完全な置換というより **比較用・代替用バックエンド** と見るのが安全です。
