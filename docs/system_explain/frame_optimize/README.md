# `frame_optimize/`

フレーム最適化サブシステムのシステム説明です。現行運用では **EasyStart → `examples/run_frame_optimization.m` → `matlab/run_canonical_frame_optimization.m` → `frameObjectiveFunction.m` → TCP FEM backend** の順に流れます。

## 読み順

1. [examples/README.md](examples/README.md)
2. [examples/run_frame_optimization.m.md](examples/run_frame_optimization.m.md)
3. [matlab/README.md](matlab/README.md)
4. [matlab/run_canonical_frame_optimization.m.md](matlab/run_canonical_frame_optimization.m.md)
5. [matlab/frameObjectiveFunction.m.md](matlab/frameObjectiveFunction.m.md)
6. [cpp/README.md](cpp/README.md)

## 現在の要点

- `examples/run_frame_optimization.m` は interactive wrapper
- `run_canonical_frame_optimization.m` が非対話の正規 API
- `runFrameOptimization.m` は旧 API 互換ラッパー
- `generate_onshape_3d_sketch.m` は Onshape 向け centerline-only exporter
- downstream へは unified vehicle artifact と `K_chassis` mirror で繋ぐ
