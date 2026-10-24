# test_nn_tire_integration.cpp

- **目的**: NN タイヤモデルのロード、推論一貫性、runtime postprocess、熱モデルを検証します。
- **主要要素**: `NNTireModelTest`、`RealNNModel`、`TireThermalTest`。
- **理論背景**: 学習済みモデルの C++ 推論再現性と温度更新の基本挙動を確認します。
- **依存**: `gtest`、`nn_lookup_tire.hpp`、`tire_thermal_integrator.hpp`。
- **システム上の位置づけ**: NN/PINN タイヤ導入時の統合回帰です。

