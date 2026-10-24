# frame_optimize/matlab/

フレーム最適化の MATLAB 本体です。現行の正規フローは **`run_canonical_frame_optimization.m` を中心に、artifact 解決・GA 実行・TCP FEM 評価・artifact 保存** を組み立てています。

## ファイル

- [run_canonical_frame_optimization.m.md](./run_canonical_frame_optimization.m.md)
- [runFrameOptimization.m.md](./runFrameOptimization.m.md)
- [frameObjectiveFunction.m.md](./frameObjectiveFunction.m.md)
- [generate_onshape_3d_sketch.m.md](./generate_onshape_3d_sketch.m.md)
- [checkFrameCompliance.m.md](./checkFrameCompliance.m.md)
- [beam_fem_server_manager.m.md](./beam_fem_server_manager.m.md)
- [beam_fem_solver_tcp_java.m.md](./beam_fem_solver_tcp_java.m.md)
- [get_fem_server_host_port.m.md](./get_fem_server_host_port.m.md)
- [generate_adaptive_topology.m.md](./generate_adaptive_topology.m.md)
- [generate_load_cases.m.md](./generate_load_cases.m.md)
- [checkExclusionZones.m.md](./checkExclusionZones.m.md)
- [loadMaterials.m.md](./loadMaterials.m.md)
- [loadPipeCatalog.m.md](./loadPipeCatalog.m.md)
- [generate_parametric_nodes.m.md](./generate_parametric_nodes.m.md)
- [load_suspension_hardpoints.m.md](./load_suspension_hardpoints.m.md)
- [load_rear_suspension_hardpoints.m.md](./load_rear_suspension_hardpoints.m.md)
- [resolve_suspension_geometry_source.m.md](./resolve_suspension_geometry_source.m.md)
- [verify_suspension_geometry.m.md](./verify_suspension_geometry.m.md)

## 読む順番

1. `run_canonical_frame_optimization.m`
2. `frameObjectiveFunction.m`
3. `generate_onshape_3d_sketch.m`
4. `generate_parametric_nodes.m`
5. `generate_adaptive_topology.m`
6. `generate_load_cases.m`
7. `resolve_suspension_geometry_source.m`
8. `beam_fem_solver_tcp_java.m`
9. `beam_fem_server_manager.m`
10. `runFrameOptimization.m`
