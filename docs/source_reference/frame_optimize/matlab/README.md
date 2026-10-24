# `frame_optimize/matlab/`

## 役割

フレーム最適化の MATLAB 本体です。現行の核は `run_canonical_frame_optimization.m` と `frameObjectiveFunction.m` で、artifact 解決、GA、荷重条件、solver 接続、保存をここで組み立てます。

## 主なファイル

- [run_canonical_frame_optimization.m.md](run_canonical_frame_optimization.m.md)
- [runFrameOptimization.m.md](runFrameOptimization.m.md)
- [frameObjectiveFunction.m.md](frameObjectiveFunction.m.md)
- [generate_onshape_3d_sketch.m.md](generate_onshape_3d_sketch.m.md)
- [resolve_suspension_geometry_source.m.md](resolve_suspension_geometry_source.m.md)
- [beam_fem_solver_tcp_java.m.md](beam_fem_solver_tcp_java.m.md)
- [beam_fem_server_manager.m.md](beam_fem_server_manager.m.md)
