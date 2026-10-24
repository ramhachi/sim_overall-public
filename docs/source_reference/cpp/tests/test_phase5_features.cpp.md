# test_phase5_features.cpp

- **目的**: Ackermann、compliance steer、ARB、LuGre、適応タイムステップの Phase 5 機能を総合検証します。
- **主要要素**: `Phase5Test` fixture の多数の `TEST_F`。
- **理論背景**: 幾何学・補正舵角・ロール剛性・高剛性摩擦・イベント時サブステップの動作確認です。
- **依存**: `gtest`、`vehicle_14dof_new.hpp`。
- **システム上の位置づけ**: Phase 5 実装の総合回帰ファイルです。

