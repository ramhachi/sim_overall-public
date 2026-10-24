# `frame_optimize/README.md`

フレーム最適化サブシステムの **source-first 索引**です。現行の正規経路は **EasyStart → `examples/run_frame_optimization.m` → `matlab/run_canonical_frame_optimization.m` → `frameObjectiveFunction.m` → TCP FEM backend** です。

## いま有効な実行経路

1. `EasyStart.m` のフレーム最適化ランチャー
2. `frame_optimize/examples/run_frame_optimization.m`
3. `frame_optimize/matlab/run_canonical_frame_optimization.m`
4. `frame_optimize/matlab/frameObjectiveFunction.m`
5. `frame_optimize/matlab/beam_fem_solver_tcp_java.m`
6. `frame_optimize/cpp/beam_fem_server.cpp`

`frame_optimize/matlab/runFrameOptimization.m` は残っていますが、現在は **旧呼び出し互換ラッパー**です。

## このサブシステムがしていること

- authoritative vehicle artifact からサスペンション hardpoints を解決する
- hardpoints と形状比率からフレーム節点を生成する
- 距離しきい値からトポロジー候補を張る
- 管材カタログからパイプ断面を選ぶ
- TCP FEM サーバで質量・ねじり剛性・変位を評価する
- CG・除外ゾーン・材料制約・疎性を罰則化して最適化する
- 結果を unified vehicle artifact JSON に書き戻す
- 必要に応じて SolidWorks VBA / Onshape FeatureScript の CAD 出力を生成する

## 実装上の見取り図

| 層 | 主なファイル | 役割 |
| --- | --- | --- |
| 起動ラッパー | `examples/run_frame_optimization.m` | 対話入力を受けて canonical API を呼ぶ |
| Canonical API | `matlab/run_canonical_frame_optimization.m` | 非対話で正規フロー全体を実行する |
| 互換入口 | `matlab/runFrameOptimization.m` | 旧 API 呼び出しを canonical objective へ橋渡しする |
| CAD exporter | `matlab/generate_solidworks_macro.m`, `matlab/generate_onshape_3d_sketch.m` | 結果を CAD 向け macro / script に変換する |
| MATLAB 本体 | `matlab/frameObjectiveFunction.m` ほか | 幾何生成、荷重条件生成、目的関数評価 |
| Solver client | `matlab/beam_fem_solver_tcp_java.m` | TCP FEM サーバとの接続と再試行 |
| Solver lifecycle | `matlab/beam_fem_server_manager.m` | ローカル / Docker サーバの起動停止 |
| C++ TCP | `cpp/beam_fem_server.cpp` | 通常運用の FEM サーバ |
| C++ CLI | `cpp/beam_fem_standalone.cpp` | JSON 入出力の単発解析 |
| テスト | `tests/` | スモーク、回帰、resolver fail-fast の確認 |

## 読み順

1. [examples/run_frame_optimization.m.md](examples/run_frame_optimization.m.md)
2. [matlab/run_canonical_frame_optimization.m.md](matlab/run_canonical_frame_optimization.m.md)
3. [matlab/frameObjectiveFunction.m.md](matlab/frameObjectiveFunction.m.md)
4. [matlab/generate_onshape_3d_sketch.m.md](matlab/generate_onshape_3d_sketch.m.md)
5. [matlab/generate_parametric_nodes.m.md](matlab/generate_parametric_nodes.m.md)
6. [matlab/beam_fem_solver_tcp_java.m.md](matlab/beam_fem_solver_tcp_java.m.md)
7. [cpp/beam_fem_server.cpp.md](cpp/beam_fem_server.cpp.md)

## 現在の注意点

- canonical flow は **artifact-first** です。`FL` / `RL` hardpoints が取れない場合は fail-fast に寄せています。
- `params.frame` へ詳細結果を保存しつつ、`K_chassis` も mirror して下流へ効かせます。
- MEX backend は残っていますが、通常運用は TCP サーバ版が優先です。
- Onshape 向け出力は **centerline-only** で、tube solid / cope cut / end treatment は intentionally out-of-scope です。

## 主なファイル

- [examples/run_frame_optimization.m.md](examples/run_frame_optimization.m.md)
- [matlab/run_canonical_frame_optimization.m.md](matlab/run_canonical_frame_optimization.m.md)
- [matlab/runFrameOptimization.m.md](matlab/runFrameOptimization.m.md)
- [matlab/frameObjectiveFunction.m.md](matlab/frameObjectiveFunction.m.md)
- [matlab/generate_onshape_3d_sketch.m.md](matlab/generate_onshape_3d_sketch.m.md)
- [matlab/resolve_suspension_geometry_source.m.md](matlab/resolve_suspension_geometry_source.m.md)
- [matlab/beam_fem_server_manager.m.md](matlab/beam_fem_server_manager.m.md)
- [matlab/beam_fem_solver_tcp_java.m.md](matlab/beam_fem_solver_tcp_java.m.md)
- [matlab/get_fem_server_host_port.m.md](matlab/get_fem_server_host_port.m.md)
- [matlab/generate_adaptive_topology.m.md](matlab/generate_adaptive_topology.m.md)
- [matlab/generate_load_cases.m.md](matlab/generate_load_cases.m.md)
- [matlab/checkExclusionZones.m.md](matlab/checkExclusionZones.m.md)
- [matlab/loadMaterials.m.md](matlab/loadMaterials.m.md)
- [matlab/loadPipeCatalog.m.md](matlab/loadPipeCatalog.m.md)

## サブディレクトリ

- [examples/README.md](examples/README.md)
- [matlab/README.md](matlab/README.md)
- [cpp/README.md](cpp/README.md)
- [mex/README.md](mex/README.md)
- [tests/README.md](tests/README.md)
