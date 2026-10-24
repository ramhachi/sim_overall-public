# mpc_driver.hpp

- **目的**: sampling-based preview MPC 経路追従ドライバーを提供します。
- **主要要素**: `MPCStats`、`MPCPathFollowingDriver`。
- **理論背景**: 操舵角・縦加速度の候補を有限ホライズンで予測評価し、最小コストの最初の制御入力のみを適用する receding horizon 制御です。
- **依存**: `track_distance.hpp`、`track_model.hpp`、`velocity_profile.hpp`、`vehicle_state.hpp`、`mpc_problem.hpp`。
- **システム上の位置づけ**: OSQP 非依存で常時使える MPC サブシステムの標準版です。
