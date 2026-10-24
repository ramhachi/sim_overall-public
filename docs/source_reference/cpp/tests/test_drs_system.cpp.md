# test_drs_system.cpp

- **目的**: DRS 状態遷移、安全条件、統計、空力減少統合を検証します。
- **主要要素**: `DRSSystemTest`、`DRSIntegrationTest`。
- **理論背景**: 速度・横 G・舵角条件と起動遅延が正しく働くことを確認します。
- **依存**: `gtest`、`drs_system.hpp`、`aero_map_4d.hpp`。
- **システム上の位置づけ**: DRS 安全ロジックの回帰です。

