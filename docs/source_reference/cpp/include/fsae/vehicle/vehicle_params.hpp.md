# vehicle_params.hpp

- **目的**: 車両全体の質量、サスペンション、タイヤ、操舵、駆動、ブレーキ、空力パラメータを一元化します。
- **主要要素**: `WheelIndex`、`MassParams`、`SuspensionParams`、`TireParams`、`SteeringParams`、`PowertrainParams`、`BrakeParams`、`AeroParams`、`VehicleParams`。
- **理論背景**: 14DOF モデルだけでなく PID/MPC/空力/タイヤ熱まで共有される基底パラメータ群です。
- **依存**: `nlohmann/json`、`array`、`fstream`。
- **システム上の位置づけ**: ほぼすべての物理・制御モジュールが参照する設定の中心です。

