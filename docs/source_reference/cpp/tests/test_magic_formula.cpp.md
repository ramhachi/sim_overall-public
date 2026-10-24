# test_magic_formula.cpp

- **目的**: Magic Formula の純スリップ、複合スリップ、SAT、荷重依存、有限値性を検証します。
- **主要要素**: `MagicFormulaTest` fixture と各 `TEST_F`。
- **理論背景**: MF5.2 実装が基本的な S 字特性と複合スリップ低減を満たすかを見ます。
- **依存**: `gtest`、`magic_formula.hpp`。
- **システム上の位置づけ**: 高忠実度タイヤモデル導入時の回帰テストです。

