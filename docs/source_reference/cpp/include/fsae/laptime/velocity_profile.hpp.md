# velocity_profile.hpp

- **目的**: トラック曲率と車両性能から目標速度プロファイルを生成します。
- **主要要素**: `VelocityPoint`、`VehiclePerformance`、`VelocityProfileGenerator`。
- **理論背景**: 旋回限界、前向き加速積分、後向き減速積分、必要に応じた空力マップ補正を組み合わせます。
- **依存**: `track_model.hpp`、`aero_map_4d.hpp`、`optional`。
- **システム上の位置づけ**: PID/MPC/ラップタイム計算の速度目標生成器です。

