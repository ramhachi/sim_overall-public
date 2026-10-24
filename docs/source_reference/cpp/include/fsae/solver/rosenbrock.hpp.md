# rosenbrock.hpp

- **目的**: Rosenbrock23 法による剛性 ODE 積分器を提供します。
- **主要要素**: `Rosenbrock23Coeffs`、`RosenbrockConfig`、`SolverResult`、`Rosenbrock23<N>`、`RosenbrockDynamic`。
- **理論背景**: ヤコビアンを使う半陰的 2/3 次スキームで、誤差推定に基づき適応ステップも行います。
- **依存**: `Eigen::LU`、`functional`。
- **システム上の位置づけ**: RK4 より剛性に強い代替積分法です。

