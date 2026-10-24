# Performance Note: Lightweight Simulator Plan (2026-06-15)

## 目的

学習と車両諸元最適化で使う評価器を、現行の「高忠実度 14DOF 統合系」から切り分け、
目安として **1 ラップ 1 s 未満** を狙える運用へ移すための整理メモ。

## 実測

2026-06-15 に host binary で測定。

- `build/cpp/laptime_sim --track autocross --driver pid --output_json`
  - wall-clock: 約 `6.35 s`
  - simulated lap time: `77.222 s`
  - tire runtime: `tmeasy`
- `build/cpp/laptime_sim --track skidpad --driver pid --output_json`
  - wall-clock: 約 `0.565 s`
  - simulated lap time: `15.342 s`
- `build/cpp/laptime_sim --track acceleration --driver pid --output_json`
  - wall-clock: 約 `0.254 s`
  - simulated run time: `6.27 s`
- `build/cpp/laptime_sim --track autocross --driver mpc --output_json`
  - wall-clock: 約 `61.4 s`
  - failed to complete lap within `117.033 s` simulated time
- `build/cpp/laptime_sim --track autocross --driver attack --output_json`
  - `20 s` timeout以内で完走せず

参考として 14DOF 単体デモは軽い。

- `build/cpp/fsae_14dof step_steer`: `5 s` simulated / `16 ms` compute
- `build/cpp/fsae_14dof circle`: `10 s` simulated / `29 ms` compute

つまり重いのは「14DOF ODE そのもの」より、**ラップ完走用の統合ループ**と
**高機能ドライバー**を含んだ経路。

## なぜ重いか

### 1. 1 ms の 14DOF RK4 をラップ全体で回している

`Vehicle14DOF_New::step()` は 1 ステップごとに RK4 を 4 回評価する。

- `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
  - `step()` で RK4: lines 152-188
  - `step_adaptive()` で 2x/5x サブステップ化: lines 229-262

autocross `77.222 s` を `dt = 0.001 s` で積分すると:

- main steps: 約 `77,222`
- derivative evaluations: 約 `308,888`
- wheel-force evaluations: 約 `1.24M`
- K&C load iteration が 2 回入る輪では約 `2.47M` 相当

この規模は RL や GA の inner loop としてはまだ重い。

### 2. 各微分評価の中身が重い

`compute_derivatives()` は各 wheel で次をまとめて計算する。

- full 3D rotation
- suspension / Bouc-Wen / LuGre / bump stop
- K&C load-sensitive camber/toe fixed-point
- tire slip lag
- tire model (`NN -> MF -> TMeasy`)

参照:

- `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
  - main derivative body: lines 986-1290
  - K&C fixed-point iteration: lines 1129-1139
  - tire model branch: lines 1163-1287

つまり 1 回の ODE 評価が「単なる bicycle model」ではない。

### 3. RL 学習パスも同じ重さを抱えている

RL server は 20 ms ごとに 1 回 action を受けるが、内部では 1 ms × 20 substeps を回す。

- `cpp/src/fsae_rl_server.cpp`
  - `N_SUBSTEPS = 20`, `SIM_DT = 0.001`, `CONTROL_DT = 0.020`: lines 54-56
  - `Vehicle14DOF_New` を保持: line 195
  - action ごとに 20 substeps: lines 231-266

したがって学習が重い理由は、通信ではなく **plant fidelity が高すぎる**こと。

### 4. MPC / attack driver は予測ロールアウトも多い

MPC:

- steering candidates 7 個 × accel candidates 5 個
- 各 candidate で horizon `N = 20`
- 1 control update あたり `7 * 5 * 20 = 700` rollout steps

参照:

- `cpp/include/fsae/control/mpc_problem.hpp`: `N = 20`, lines 28-31
- `cpp/include/fsae/driver/mpc_driver.hpp`
  - candidate sweep: lines 184-186
  - rollout horizon loop: lines 431-436

attack:

- primitives 7 個 × speed scales 4 個
- 各 candidate で horizon `N = 20`
- 1 control update あたり `7 * 4 * 20 = 560` rollout steps

参照:

- `cpp/include/fsae/driver/attack_driver.hpp`
  - primitive/speed sweep: lines 83-113
  - primitive definitions: lines 776-790
  - rollout horizon loop: lines 1046-1053

40 Hz (`0.025 s`) で 77 s ラップを走ると、MPC/attack は百万オーダーの
予測ステップを追加で踏む。これは PID より桁違いに重い。

### 5. track nearest query も軽くはない

`TrackDistanceQuery::nearest()` は hint 付き局所探索に加えて、
毎回 10 分割の精密化を行う。

- `cpp/include/fsae/track/track_distance.hpp`: lines 68-135

PID では許容範囲だが、MPC/attack rollout の内側で大量に呼ぶと効いてくる。

### 6. PINN を使うとさらに重くなる

現行実測は `tmeasy` だった。PINN では wheel ごとに NN forward が入る。

- `cpp/include/fsae/physics/tire/nn_lookup_tire.hpp`
  - runtime layer forward: lines 314-357
  - physical forward path: lines 385-405

将来的に PINN 主系統で評価器を回すと、現状よりさらに重くなる見込み。

### 7. MATLAB / Docker orchestration も評価回数が多いと効く

最適化や感度分析の一部は params JSON を書き、外部プロセスを起動している。

- `matlab/VehicleSimRunner.m`: params file write + command launch, lines 116-145

1 本 5-6 s の本体に対しては副次的だが、1 s 未満を目標にすると
起動オーバーヘッドも無視しにくい。

## 結論

現行シミュレータが学習や最適化に重い主因は次の 3 点。

1. **14DOF full transient plant を 1 ms RK4 で最後まで回していること**
2. **各 ODE 評価に tire + K&C + suspension nonlinearity が全部入っていること**
3. **MPC/attack や RL でも、その高忠実度 plant を内側で大量に使っていること**

## 1 秒未満を狙うための現実的な方針

### 方針A: 学習/最適化用 backend を 14DOF から分離する

最重要。`full_eval` と `fast_eval` を分ける。

- `fast_eval`
  - 3DOF or 6DOF
  - `dt = 0.01 ~ 0.02 s`
  - semi-implicit Euler or RK2
  - quasi-static load transfer
  - tire は table lookup
  - frame twist / Bouc-Wen / LuGre / bump stop / detailed K&C / tire thermal を切る
- `full_eval`
  - 現行 14DOF を validation 専用に残す

repo には既に軽量モデル資産がある。

- `matlab/vehicle_3dof/vehicle_3dof_model.m`
- `matlab/vehicle_6dof/vehicle_6dof_model.m`
- `cpp/include/fsae/vehicle/bicycle_model.hpp`

### 方針B: tire を offline table 化する

学習/最適化用途では、online に MF/PINN を毎回評価せず、
`(SR, SA, Fz, IA, T)` あるいは簡略化した 3-4 次元格子を事前生成して補間する。

候補:

- `Fx, Fy, Mz` bilinear / trilinear lookup
- combined-slip ellipse を直接 table 化
- 温度は fixed or slow state として別扱い

これは fast backend の効果が大きい。

### 方針C: closed-loop lap sim でも controller を簡略化する

学習や車両諸元探索ではまず PID または simple pursuit に固定する。

- `mpc` / `attack` は設計比較・最終検証だけ
- inner-loop の設計探索に predictive driver を使わない

今の実測でも PID と MPC の差が大きい。

### 方針D: 1 評価 1 プロセス起動をやめる

最適化・感度分析では persistent runner を使う。

- C++ 側で in-process batch evaluator
- もしくは RL server と同様の常駐 evaluator
- MATLAB からは socket / pipe 経由で params を渡す

1 s を切り始めると起動コストが相対的に無視できなくなる。

### 方針E: early reject を強くする

最適化用途では完走不能候補を最後まで積分しない。

- off-track excess が閾値超え
- speed / progress が明らかに不足
- spin / yaw rate / slip が閾値超え
- objective lower bound で枝刈り

GA / BayesOpt / RL ではこの pruning が効く。

## 推奨ロードマップ

### Step 1

`fast_eval` を新設する。

- C++ で bicycle/6DOF ベース
- same vehicle params JSON を入力
- same track / lap result schema を出力
- driver は PID / RL-friendly simple controller のみ

目標:

- autocross `0.1 ~ 0.5 s / lap`

### Step 2

最適化・感度分析・RL 学習を `fast_eval` に切り替える。

- generation/episode の大半は `fast_eval`
- 上位 `K` 候補のみ `full_eval`

### Step 3

`full_eval` も必要なら高速化する。

- K&C iteration を option 化
- tire thermal / LuGre / Bouc-Wen の on/off
- per-wheel rotation / nearest query の削減
- track query / line geometry cache の強化

## 優先順位

優先度順では以下。

1. **評価 fidelity の階層化 (`fast_eval` / `full_eval`)**
2. **tire の table lookup 化**
3. **predictive driver を inner loop から外す**
4. **persistent evaluator 化**
5. **14DOF path 自体の micro-optimization**

5 は必要だが、1-4 より効果が小さい可能性が高い。

## 判断

「現行の 14DOF 統合ラップシミュレータをそのまま 1 ラップ 1 s 未満へ」
は不可能ではないが、労力の割に筋が悪い。

最も現実的なのは:

- **学習・最適化用に別 backend を切る**
- **14DOF は最終確認専用にする**

この分離が、速度・保守性・物理整合のバランスが最も良い。
