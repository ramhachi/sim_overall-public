# vehicle_14dof_new.hpp

## 1. ファイルの目的
現行の 14DOF フル車両モデルを定義する中核ヘッダです。ばね上 6DOF、ばね下 4DOF、車輪回転 4DOF に加え、タイヤ緩和長、Bouc-Wen、LuGre、フレームねじり、ステアリングホイール、DRS、複数タイヤモデル、4D エアロを統合しています。

## 2. 実行上の役割
シミュレーション本体で毎ステップ呼ばれ、車両全状態の微分計算と RK4 積分を行います。PID/MPC/RL ドライバーの出力は最終的にこのクラスへ入力されます。

## 3. 依存関係
- `vehicle_params.hpp`, `vehicle_state.hpp`, `drs_system.hpp`, `steering_torque.hpp`
- `physics/tire/*`: TMeasy、NN、Magic Formula、温度モデル
- `physics/suspension/*`: Bouc-Wen、LuGre
- `physics/powertrain/*`: エンジントルクマップ、LSD
- `physics/aero/aero_map_4d.hpp`

## 4. 入力/出力
- 入力: `DriverInput`, `dt`, 初期速度、車両パラメータ
- 出力: 更新後 `State14DOF`
- 補助出力: タイヤ力、サスペンション力、POU 変位、フレームねじり、ステアリングホイール状態など多数

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct State14DOF`
- 位置姿勢: `x,y,z,phi,theta,psi`
- 速度: `vx,vy,vz,p,q,r`
- ばね下: `z_u[4]`, `vz_u[4]`
- 車輪回転: `omega[4]`
- タイヤ緩和状態: `tire_xi[4]`, `tire_eta[4]`
- サスペンション履歴: `damper_z[4]`, `friction_z[4]`
- 追加状態: `phi_twist`, `dphi_twist`, `delta_sw`, `ddelta_sw`
- `SIZE = 44`
- `speed()`, `sideslip()`: 便利関数

### `class Vehicle14DOF_New`
#### 公開 API
- `using Params = VehicleParams`
- `FL/FR/RL/RR`: 車輪インデックス定数
- `Vehicle14DOF_New(const Params&)`: タイヤ・サスペンション・パワートレイン・エアロ・平衡位置を初期化し、選択タイヤモデルをロード、`reset()` も実行
- `state() const / state()`, `params()`, `time()`, `z_equilibrium()`
- `drs_system()`, `is_drs_active()`: DRS 参照
- `set_initial_velocity(double)`: 初速と車輪角速度設定
- `reset()`: 44 状態を初期化し、平衡位置へ戻し、キャッシュ更新
- `step(double, const DriverInput&)`: DRS 更新、RK4 積分、タイヤ変形飽和、必要時タイヤ温度更新
- `step_adaptive(...)`: バンプストップ接触や LuGre 高剛性時にサブステップ分割
- デバッグ getter: `get_tire_Fz/Fx/Fy/Mz`, `get_bw_forces`, `get_lugre_forces`, `get_spring_forces`, `get_pou_displacement`, `get_pou_camber`, `get_frame_twist`, `get_frame_twist_rate`, `get_split_roll_angles`, `get_steering_wheel_angle`, `get_steering_wheel_rate`, `get_tire_angle`
- `compute_pou_displacement(...)`: POU 操舵ジャッキアップ変位計算
- `eval_quadratic(...)`, `is_zero_polynomial(...)`: K&C 近似式用 helper
- `compute_camber_angle(...)` 2 版: K&C/POU/ロール/荷重を反映したキャンバー計算
- `get_engine_rpm(int)`, `get_engine_torque(int,double)`: デバッグ用パワートレイン参照
- `compute_ackermann_steering(...)`: アッカーマン操舵角分配
- `compute_compliance_steer(...)`: SAT によるコンプライアンスステア
- `compute_arb_forces(...)`: 前後 ARB 力

#### 非公開 helper
- `to_lower(...)`: 文字列小文字化
- `initialize_selected_tire_model()`: `model_type` に応じて PINN / MF / TMeasy を選択
- `setup_tire_params()`, `setup_bouc_wen_params()`, `setup_lugre_params()`, `setup_powertrain()`, `setup_aero_map()`
- `compute_equilibrium()`: 静荷重と平衡位置
- `state_to_vector(...)`, `vector_to_state(...)`: 44 次元変換
- `get_wheel_positions()`: 4 輪位置
- `compute_rotation_matrices(...)`: 車体系回転行列と Euler rate 変換
- `compute_bumpstop_force(...)`
- `compute_split_roll_angles(...)`: フレームねじりを前後ロール角へ分配
- `compute_total_steering(...)`: ドライバー入力 + コンプライアンス + アッカーマン統合
- `compute_toe_angle(...)` 2 版: K&C/荷重依存トー角計算
- `compute_effective_relaxation_length(...)`: 荷重・温度依存の緩和長
- `compute_roll_center_height_for_wheel(...)`
- `compute_pitch_force_arm_for_wheel(...)`
- `compute_effective_jacking_force(...)`
- `compute_wheel_rotation_matrix(...)`: ステア・トー・キャンバーを含む座標変換
- `compute_suspension_roll_moments(...)`: 前後別ロールモーメント
- `compute_derivatives(...)`: 全サブシステム統合の中心関数
  - ステアリング入力決定（角度モード/トルクモード）
  - フレームねじりによる前後ロール角分離
  - 各輪で POU、Bouc-Wen、LuGre、バンプストップ、K&C、ジャッキング、タイヤ力を計算
  - タイヤモデルは NN → Magic Formula → TMeasy の順に選択
  - ばね下運動、合力・合モーメント、4D エアロ、重力、完全オイラー方程式、LSD 付き駆動・ブレーキ、フレームねじり、ステアリングホイール運動を更新
