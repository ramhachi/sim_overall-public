# tire_thermal_integrator.hpp

- **目的**: 4 輪分のタイヤ表面温度を時間積分するクラスを提供します。
- **主要要素**: `TireThermalParams`、`TireSlipInfo`、`TireThermalIntegrator`。
- **理論背景**: 発熱、対流、路面伝導をオイラー積分し、表面温度を更新します。
- **依存**: `array`、`algorithm`。
- **システム上の位置づけ**: NN タイヤモデル使用時の温度状態管理に使われます。

