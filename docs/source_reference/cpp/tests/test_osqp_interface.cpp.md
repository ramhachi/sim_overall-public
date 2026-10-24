# test_osqp_interface.cpp

- **目的**: OSQP ラッパーの求解、ウォームスタート、更新 API、利用不可時の分岐を検証します。
- **主要要素**: `OSQPInterfaceTest` fixture。
- **理論背景**: 単純 QP を通して C API ラッパーの正当性を確認します。
- **依存**: `gtest`、`osqp_interface.hpp`。
- **システム上の位置づけ**: MPC の求解基盤を守るテストです。

