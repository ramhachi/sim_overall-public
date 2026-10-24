# test_pou.cpp

- **目的**: POU（Pushrod on Upright）由来のジャッキアップと操舵誘発キャンバーを検証します。
- **主要要素**: `POUKinematicsTest`、`POUIntegrationTest`。
- **理論背景**: 操舵角に対する車高・キャンバー変化の多項式近似と統合効果を評価します。
- **依存**: `gtest`、`vehicle_14dof_new.hpp`。
- **システム上の位置づけ**: POU 幾何近似の回帰テストです。

