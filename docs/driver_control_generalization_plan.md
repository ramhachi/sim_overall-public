# Driver Control Generalization Plan

## 目的

この計画の目的は、PID や RL の優劣を決めることではなく、車両諸元が変わっても柔ボディ・ステアトルクモードで実ドライバーに近いラップタイムを出せる汎用ドライバー制御器を作ることです。

PID、MPC、RL は手段として扱い、最終的な評価対象は「車両を理解して、限界付近を安定して走れる driver stack」です。

## パラメータチューニング以外にやるべきこと

### 1. ドライバーの契約を固定する

車両モデルとドライバーの間で、入力・出力・評価指標を固定します。

- 入力: 車両状態、Frenet 誤差、曲率 preview、トラック幅、タイヤ/車両パラメータ、ステア状態、フレーム捻り状態
- 出力: steering torque、throttle、brake、gear、DRS request
- 評価: lap time、off-track、横偏差、heading error、steering saturation、steering lag、frame twist、制御滑らかさ

これにより、PID/RL/MPC を差し替えても同じ土俵で比較できます。

### 2. アクチュエータ制約込みの速度 planner を作る

現状の速度 profile は摩擦限界寄りで、ステアトルク系の遅れを十分に見ていません。

次に必要なのは、各地点で以下を見て速度上限を決める planner です。

- 必要操舵角と現在の `delta_sw`
- `T_driver_max` と操舵トルク飽和
- ステア応答時間
- `phi_twist`, `dphi_twist`
- トラック境界までの余裕
- 制動距離

これは単なる `target_speed *= const` ではなく、車両ごと・地点ごとに変わる feasible speed envelope にするべきです。

### 3. ステアトルク内側制御を独立部品化する

トルク操舵では、経路追従器が直接ステアトルクを決めるべきではありません。

望ましい分離は以下です。

- 外側: 目標 yaw rate / 目標タイヤ舵角 / 目標曲率を決める
- 内側: `delta_sw` と `ddelta_sw` を見て steering torque を出す

内側制御には、摩擦、SAT、トルク飽和、anti-windup、rate limit を明示的に持たせます。RL を使う場合も、まずこの内側制御器を安定基準として持つべきです。

### 4. ライン planner を導入する

センターライン追従だけでは、実ドライバー相当のタイムには届きません。

次の段階では、トラック幅を使って以下を planner に入れます。

- out-in-out
- 曲率最小化ライン
- 入口速度と出口速度のトレードオフ
- off-track margin
- 車両諸元ごとの最適ライン差

まずは単純な boundary-constrained smoothing line から始め、後で最適制御やMPCへ拡張します。

### 5. 固定ゲインではなく vehicle-adaptive controller にする

「どんな諸元でも走る」には、各車両で手作業チューニングしないことが重要です。

必要なのは、車両パラメータから制御器内部の基準値を自動計算する仕組みです。

- wheelbase から曲率 FF を設定
- `R_steer`, `I_steer`, `C_steer_torque`, `T_driver_max` から内側ループ帯域を設定
- tire mu から速度 envelope を設定
- `K_chassis`, `C_chassis`, `I_twist` から捻りリスクを設定

ゲインを直接最適化するより、物理パラメータから controller schedule を作る方向に寄せます。

### 6. 車両諸元 sweep を評価ゲートにする

単一車両で速いだけでは不十分です。

最低限、以下をランダムまたは格子で振って評価します。

- mass, CG, wheelbase
- tire mu, tire radius
- aero CL/CD
- `K_chassis`, `C_chassis`, `I_twist`
- steering ratio, steering inertia, friction, damping, torque limit
- suspension spring/ARB

各サンプルで、再チューニングなしに完走率・ラップタイム・制御健全性を見るべきです。

### 7. RL は直接操作ではなく補助から始める

RL を最終的に使うとしても、最初から steering torque / throttle / brake を全部任せるのは難しすぎます。

現実的な順序は以下です。

1. PID/MPC を教師にして imitation / DAgger
2. RL は速度補正、ライン補正、残差トルク補正から始める
3. domain randomization で車両諸元を振る
4. 最後に直接操作 policy と比較する

RL は「運転の全責任者」ではなく、まず「model-based driver の弱点を補う部品」として扱います。

### 8. 成功条件を lap time だけにしない

高速だが off-track している走行や、ステアトルク飽和に張り付く走行は失敗です。

今後の合格条件は少なくとも以下にします。

- lap completed
- no off-track
- max lateral error が track width 内
- steering saturation ratio が一定以下
- steering lag ratio が一定以下
- frame twist / twist rate が一定以下
- 複数車両諸元で再チューニングなしに成立

## 次に実施する実験

### Experiment A: vehicle parameter sweep baseline

現在の torque PID を固定し、車両諸元を振って失敗地図を作ります。

目的は、何をチューニングすべきかではなく、どの物理条件で driver stack が破綻するかを把握することです。

### Experiment B: actuator-aware speed envelope

速度 planner にステア遅れ・トルク飽和・制動距離を入れます。

期待する結果は、手作業の `torque_speed_scale` に頼らず、各車両で安全な上限速度が出ることです。

### Experiment C: track-boundary line planner

センターラインではなく、トラック幅内の走行ラインを生成します。

最初は曲率を滑らかにする幾何 planner とし、後で速度 planner と結合します。

### Experiment D: steering inner-loop benchmark

ステップ操舵・S字操舵・SATあり条件で、目標 `delta_sw` に対する追従性を評価します。

ここで得た帯域と飽和特性を速度 planner に戻します。

### Experiment E: hybrid driver comparison

同じ vehicle sweep に対して以下を比較します。

- torque PID
- torque PID + actuator-aware speed planner
- torque PID + line planner
- residual RL
- direct RL

比較指標は lap time だけでなく、完走率と制御健全性を含めます。

## 当面やらないこと

- 各車両ごとの手作業ゲイン合わせ
- off-track 走行を成功扱いする最速化
- 旧角度操舵 actor をトルク操舵へ流用すること
- 単一車両だけでRLの良否を判断すること

## 判断基準

次のマイルストーンは、柔ボディ・ステアトルク条件で、複数の車両諸元に対して再チューニングなしで完走し、かつセンターライン基準PIDより明確に速い driver stack を作ることです。

## 2026-05-27 整合済み実装計画

### 位置付け

PID は安定フォールバック、教師、診断器として扱います。**現行の本線は LPV-LQR**（有限フレーム・トルクモードで 66.5 s / 42ケース完走）です。MPC / MPC v2 は非推奨のレガシー経路であり、新規の性能改善対象にはしません。RL は比較・探索対象として残します。

この整理により、PID にヒューリスティックを積み続ける方針から、LPV-LQR を本線に固定し、RL を比較対象として評価する方針へ移行します。

### Phase 0: RL 監視

現在走っている RL fine-tuning は止めず、完了後に exported actor を同じ比較ベンチへ投入します。

- 対象: winged flexible chassis, torque steering, `T_driver_max = 40 Nm`
- 出力先: `rl_results/autocross_winged_flexible_ft_v6_40Nm`
- 判定: RL 単体の training reward ではなく、laptime comparison gate の結果で判断

### Phase 1: Controller comparison gate

最初に固定する評価入口は `matlab/driver_generalization/run_controller_comparison_benchmark.m` です。

目的:

- PID / MPC / RL を同一 vehicle params、同一 event、同一 JSON metric で比較する
- 66 秒実測基準との差分を数値化する
- MPC v2 実装前に、既存 PID/MPC の現在地を保存する

固定条件:

- `matlab/parameter_presets/params_winged_flexible.json` の `params` を展開して使用
- `steering.torque_mode = true`
- `steering.T_driver_max = 40.0`
- `steering.T_driver_rate_max = 0.0`
- event: autocross
- reference lap time: 66.0 s

記録指標:

- completion: `lap_completed`, `failure_code`
- speed: `lap_time`, `max_speed`, `avg_speed`, `lap_time_delta_to_reference`
- path: `max_lateral_error`, `avg_lateral_error`, `max_track_limit_excess`
- steering: `steering_saturation_ratio`, `mean_abs_sw_error`, `mean_steering_lag_ratio`
- intervention: `driver_intervention_cost`, speed-loss components
- flexible body: `max_abs_phi_twist`, `max_abs_dphi_twist`
- MPC health: solve count, average solve time, max solve time, fail count

### Phase 2: PID cleanup

PID は comparison gate の結果を見ながら、次の範囲だけ直します。

- 速度制限の支配要因を見える化する
- steering lag / saturation / track margin / chassis twist の寄与を分離する
- teacher 用と attack 用の設定を分ける
- 車両性能の悪さを過剰な安全運転で隠していないか、評価指標側で分離する

PID を固定車両向け最速制御器として過剰に育てることはしません。

実装済み:

- `driver_role` を comparison gate に追加し、PID を `fallback_teacher`、MPC を `model_based_candidate`、RL を `learned_candidate` として記録する。
- `driver_intervention_cost` を後方互換の合計値として残しつつ、制御器側の介入量 `control_intervention_cost`、車両側の健康度 `vehicle_health_penalty`、低剛性などを低速走行で隠す危険 `driver_masking_risk` を分離する。
- robustness sweep の score は、車両側の twist risk を control intervention に混ぜず、別項として評価する。

> **Non-planned (2026-10-24):** Phase 3 の MPC v2 は非推奨。現行本線は LPV-LQR。以下の項目は履歴として残す。

### Phase 3: MPC v2

既存 `mpc_driver.hpp` はすぐ削除せず、v2 を横に作って比較します。v2 がベンチで勝ち、既存失敗モードを再発しないことを確認してから置換します。

MPC v2 の必須契約:

- `main_laptime_sim.cpp` が `to_map_driver_state(...)` 済みの状態を渡す契約を維持する
- ステアトルクアクチュエータ予測を入れる
- track margin、heading、steering lag、速度選択を同時に扱う
- 低速 anti-stall は既存MPCから継承する
- solve time と fail count を比較ベンチで監視する

実装済み:

- `cpp/include/fsae/driver/mpc_driver_v2.hpp` を追加し、`--driver mpc_v2` で既存MPCと横比較できる入口を作る。
- v2 は laptime 側の `to_map_driver_state(...)` 済み状態をそのまま受け取る契約を維持し、二重座標変換を行わない。
- comparison gate の標準 variant に `mpc_v2_preview` を追加し、既存 `mpc_preview` は `legacy_model_based`、v2 は `model_based_candidate` として区別する。
- 初期 v2 baseline は、長めの horizon、操舵遅れ予測有効、境界/heading/steer-rate 寄りの重みを標準化した比較対象とする。
- v2 では track margin speed cap を有効化し、境界付近で速度選択が明示的に下がるようにする。既存MPCでは同 cap を無効のまま残し、比較時に legacy と v2 の違いが `min_track_margin_speed_cap` と speed-loss 指標へ出るようにする。
- v2 では terminal progress objective を有効化し、境界安全制約だけで保守的になり過ぎないよう、予測 horizon 内の前進量不足をコストとして扱う。legacy MPC はデフォルト無効のままとし、comparison gate には `mean_mpc_rollout_progress_deficit` と `max_mpc_rollout_progress_deficit` を記録する。

### Phase 4: Robustness sweep

comparison gate で候補制御器を絞った後、車両諸元 sweep へ進みます。

- steering inertia +/-15%
- chassis stiffness / damping / twist inertia
- tire mu
- driver torque max 30 / 40 / 50 Nm
- mass / aero / front spring / front ARB

この段階で、固定車両専用の制御器を排除します。
