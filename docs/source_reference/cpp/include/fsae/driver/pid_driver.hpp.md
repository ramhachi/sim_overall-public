# pid_driver.hpp

- **目的**: Frenet 横偏差とヘディング誤差に基づく PID 経路追従ドライバーを実装します。
- **主要要素**: `PIDGains`、`PIDState`、`PIDPathFollowingDriver`、各種プリセット関数。
- **理論背景**: 外側ループで経路誤差を舵角へ変換し、必要に応じて内側トルクモード制御も行います。
- **依存**: `vehicle_14dof_new.hpp`、`track_model.hpp`、`frenet.hpp`、`velocity_profile.hpp`。
- **システム上の位置づけ**: ベースライン制御器であり、ラップタイムや MPC 比較の参照系です。

