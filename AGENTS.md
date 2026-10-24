# Build / test

```bash
mkdir -p build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
ctest -R TMeasy

./cpp/fsae_sim
./cpp/fsae_14dof step_steer
./cpp/fsae_14dof circle
./cpp/fsae_14dof acceleration
./cpp/fsae_14dof cornering

docker compose run --rm fsae-sim bash -c "cd build && cmake --build . && ctest --output-on-failure"
```

### Python

```bash
pytest tests/ -v
pytest tests/unit/test_tmeasy_model.py -v
pytest --cov=tire_fitting tests/

python3 tire_fitting/main.py \
    --drivebrake data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv \
    --cornering data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv \
    --output output/tire_params \
    --tire-name Hoosier_18x6_R20

black --line-length 100 .
ruff check .
```

### MATLAB

MATLAB の標準入口は `EasyStart.m`。  
統合実行は `matlab/run_full_pipeline.m`、サスペンション設計は `suspension_kinematics_design/`、フレーム最適化は `frame_optimize/`、PINN 関連は `tire_pinn/` を参照する。

## Architecture

### Operational pipeline

```text
EasyStart.m
  -> matlab/run_full_pipeline.m
  -> C++ executables (14DOF / laptime / RL server)
  -> output/runtime, output/laptime, output/reports
```

### Active core

- **14DOF canonical implementation**: `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
- **Simulation entrypoints**: `cpp/src/main_14dof.cpp`, `cpp/src/main_laptime_sim.cpp`
- **Tire stack**: PINN primary, with Magic Formula / TMeasy fallback
- **Optimization stack**: `suspension_kinematics_design/`, `matlab/pou_hardpoint_design/`, `frame_optimize/`

## Key conventions

- **Coordinate system**: SAE J670e — X forward, Y right, Z down
- **Euler angles**: Z-Y-X
- **Naming**: `compute_*()` for pure calculations, `update_*()` for integration-related updates
- **Solver**: RK4 at 1 ms for the main 14DOF path
- **Tire low-speed handling**: virtual velocity `v_N = 0.5 m/s`

## Key documentation

- `README.md`
- `EasyStart.m`
- `学生フォーミュラ車両運動シミュレーション総合ガイド.md`
- `docs/current_system_map.md`
- `docs/source_reference/README.md`
- `docs/future_roadmap.md`
