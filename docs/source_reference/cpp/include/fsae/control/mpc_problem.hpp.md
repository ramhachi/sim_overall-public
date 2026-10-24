# mpc_problem.hpp

- **目的**: 6DOF 自転車モデル用 MPC を OSQP 向け QP へ定式化します。
- **主要要素**: `MPCConfig`、`QPData`、`MPCProblem`。
- **理論背景**: 逐次線形化された `x_{k+1}=Ax+Bu+c`、入力制約、ステアレート制約、終端重みを疎行列で構築します。
- **依存**: `Eigen::Sparse`、`bicycle_model.hpp`、`track_distance.hpp`。
- **システム上の位置づけ**: 6DOF MPC ドライバーの中核定式化です。

