# osqp_interface.hpp

- **目的**: OSQP v1.0 C API を C++ から扱いやすく包みます。
- **主要要素**: `OSQPResult`、`OSQPConfig`、`OSQPSolverWrapper`。
- **理論背景**: 疎 QP の setup / solve / warm start / bound 更新を実行し、求解統計を返します。
- **依存**: `osqp.h`、`Eigen::Sparse`、`chrono`。
- **システム上の位置づけ**: すべての MPC ドライバーが共有する求解インターフェースです。

