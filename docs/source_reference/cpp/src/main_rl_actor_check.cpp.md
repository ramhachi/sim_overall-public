# main_rl_actor_check.cpp

- **目的**: MATLAB から書き出した RL Actor のテストデータと C++ `RLDriver` 推論結果の一致性を確認します。
- **主要要素**: `print_usage()`、`split_csv_to_doubles()`、`main()`。
- **理論背景**: 正規化済み観測と行動の最大絶対誤差を比較し、エクスポート誤差や推論実装差分を検出します。
- **依存**: `rl_driver.hpp`、`fstream`、`sstream`、`algorithm`。
- **システム上の位置づけ**: 学習済みポリシーを C++ 実機相当に移す前の検証ツールです。

