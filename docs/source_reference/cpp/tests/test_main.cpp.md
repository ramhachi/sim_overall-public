# test_main.cpp

- **目的**: Google Test ランナーのエントリポイントです。
- **主要要素**: `main()`、`InitGoogleTest`、`RUN_ALL_TESTS()`。
- **理論背景**: 個々のテスト本体ではなく、共通ランタイム起動だけを担います。
- **依存**: `gtest/gtest.h`。
- **システム上の位置づけ**: `ctest` 実行時の共通入口です。

