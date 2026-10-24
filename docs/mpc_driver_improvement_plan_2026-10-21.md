> **DEPRECATED (2026-10-24):** 現行の本線は LPV-LQR。MPC / MPC v2 は非推奨のレガシー経路であり、この計画は新規実装には使用しません。Issue #160 / #164 は not planned としてクローズ済みです。以下は当時の調査記録として残します。

# MPC/予測系ドライバー完走・高速化計画

- **作成日:** 2026-10-21
- **スコープ:** `--driver mpc` / `mpc_v2` の予測ドライバーで、逸脱せず完走し、PID 既定に劣らないラップタイムを出せるようにする。
- **位置づけ:** `docs/driver_model_improvement_plan_2026-09-28.md`(段階0〜5)のうち「予測系ドライバーの完走性」に絞って具体化したもの。ベースライン固定・物理修正(同計画の段階0/1)に取って代わるものではない。

## 1. 現状の証拠

### 1.1 本計画作成時の再現実行(2026-10-21, commit e20bf32)

コマンド: `./build/cpp/laptime_sim [--driver mpc_v2] [--params_file matlab/parameter_presets/params_torque_steering.json] --output <csv> --output-rate 0.01`
条件: トラック既定(circle r=30, 幅3 m)、初速 6 m/s、dt=1 ms、PINN タイヤ。

| 構成 | ラップタイム | 最大横偏差 | 飽和比 | 主な速度損失 |
|---|---:|---:|---:|---|
| PID(既定 params) | 8.222 s | 0.100 m | 0 | なし |
| MPC v2(角度モード) | 19.369 s | 1.084 m | 0 | なし |
| MPC v2(params_torque_steering, トルクモード) | 19.993 s | 1.087 m | 0.00035 | **track_margin(0.825 / スコア1.006)** |
| PID(params_torque_steering, トルクモード) | 24.007 s | 0.147 m | 0.0016 | 表示なし(参照値) |

ポイントは速度そのものではなく**誤差と損失の分布**。「PID の約 10 倍の横偏差」「margin cap が速度の 83 % 相当分のスコア損失を生む」のが現在の MPC v2 の姿。逸脱を margin cap で先回りして回避するたびにラップを大きく失い、それでも横偏差は 1.08 m まで膨らむ。トルク構成の PID 参照値はゲイン未最適化だが横偏差 0.147 m を守っており、問題は MPC 系の構造側にある。

### 1.2 過去の失敗ログ(リポジトリ直下のファイル)

- `mpc_v2_fail_run.csv`(トルクモード, 7.55 s): t≈1.9 s に `T_SAT_total`=+200 Nm に対し `T_driver_clamped`=+40 Nm で飽和(headroom=0)、横偏差が 1.3 m に急伸。以後は低速化で SAT が減り操舵能力は回復するが、目標速度 8〜15 m/s と実速度≈2 m/s が取り残され、s=23.38 m で進捗がほぼ停止。`throttle=1.0` のまま速度が減少し続けた。
- `mpc_v2_run.csv`(角度モード, 2.4 s): 目標速度が 16.5 m/s まで増え続け、横偏差 3.47 m でコース外まで押し出された。
- `mpc_v2_report.md`(48 s トレース): 平均速度 6.82 m/s、目標速度損失 2.56 m/s、最大横偏差 2.625 m。
- **警告:** これらのルート直下 CSV は実行コマンド/manifest が残っておらず再現不能。今後の比較は §4 の manifest 整備(P0)が前提。

## 2. 根本原因(コード裏付け)

| # | 原因 | 場所 | 説明 |
|---|---|---|---|
| R1 | **定数コマンドロールアウト** | `mpc_driver.hpp:228-240, 453-558` | 操舵 7 × 加減速 5 候補の各々を、**N=60 ステップ(1.5 s)にわたり単一の (delta, ax) 一定**で積分。「次コーナー向け制動 → アペックスからの加速」のような速度計画が候補集合に存在しないため、制動候補は w_vx の平均化的コストに打ち勝てない。速度計画そのものをロールアウトが表現できていない。 |
| R2 | **進捗コストが楽観基準で押す** | `mpc_driver.hpp:522-529` | `progress_target` はセーフティ cap(`compute_actuator_aware_target_speed`)を無視した base 速度から累積する。操舵能力が追いつかなくても「速い」候補に点が入り、飽和や margin cap を自分で誘発する速度選択にバイアスする。§1.1 の track_margin 損失 0.825 と整合。 |
| R3 | **トルクモードの操舵予測が SAT 無知** | `steering_actuator_controller.hpp:65-107`(`predict_steering_actuator_response`) | ロールアウト内の操舵予測は名目二次系のみで、**SAT 反力も T_driver_max による飽和も入っていない**。実機の内側(`compute_steering_actuator_control`, 同 149-212 行)は SAT FF と ±40 Nm 飽和を扱うが、外側のもう一段遅い応答を見ていないため「操舵遅れを過小評価 → speed プランが過剰 → 実 SAT に抑えられて逸脱」が起きる。§1.2 の fail_run はこの崩壊の典型。 |
| R4 | **authority scale が固定 clip で能力無知** | `mpc_driver.hpp:635-646` | `(T_driver_max − T_fric)/I_steer` を任意のノミナル係数 `(25 − 0.3)/0.02` と比にして clip 0.50〜1.25 で縛る。実際の有効操舵レートは横 G に伴う SAT が支配しており、±40 Nm vs SAT 200 Nm の状態はこの枠で表せない。車両・状態依存の能力を捨てている。 |
| R5 | **速度 cap がすべて反動的** | `mpc_driver.hpp:671-729` | `compute_steering_speed_cap` は現在角と必要角の gap に依存する常時反動型。`margin cap` は境界に近付いてから低速度に落とす。前方の曲率・目標線から予防的に速度を決める仕組みがなく、結果として速度を大幅に落としても横偏差を解消できていない。 |
| R6 | **操舵 FF が運動学のみ** | `mpc_driver.hpp:209-212, 663-669` | FF = `−atan(L·κ)` + アンダースチア補正と称する定数 0.002 rad/(m/s²)。実際の dFy/dα(タイヤ・荷重・速度依存)を反映せず、能力と FF のズレをコスト側の反動 cap でしか吸収できない(DRV-07 と同根)。 |
| R7 | **閉路の最近傍探索が継ぎ目で破綻する疑い** | `mpc_driver.hpp:202-203` + `track/track_distance.hpp` | hint 付き最近傍探索の継ぎ目 wrap が確認されていない(レビュー DRV 系でも指摘)。進捗計算側 `signed_track_progress`(:758-773)は wrap 済み。circle では影響小だが創作閉路コースが発火点。 |

## 3. 改修戦略

方向は「**ロールアウトに速度計画を持たせる**」ことと「**操舵能力モデルを実機に合わせる**」こと。v2 wrapper による重みや N の調整(=パラメータ列の上書き)は構造を変えないため §1.1 の病を治せないことは再現で確定済み。

介入優先度: P0(基準) → P1(速度計画) → P2(操舵モデル) → P3(前方予防) → P4(閉路+manifest) → P5(計測)。評価は各フェーズ完了単位。

### P0 — 基準実験の固定(必須の先行タスク)

- `scripts/` に gate suite 実行スクリプトを 1 本追加: circle / skidpad / straight_corner × driver {pid, mpc, mpc_v2} × 角度/トルクの最小組み合わせを走らせ、manifest(commit ハッシュ、params, タイヤ manifest, 速度プロファイル, 初期値, seed, 壁時計)とともに `output/runtime/driver_gate/<run_id>/` に保存。
- 実行は再現可能でなければならない。§1.2 のルート CSV のような「結果だけ残る」状態に戻さない。

**完了条件:** 同一スクリプトを 2 日間隔で走らせ、数値の再現性(±ε 内)を確認できる。

### P1 — ロールアウトを速度計画に対応させる(最小の構造変更)

**変更:**

1. ホライゾン内の基準速度軌道 `v_ref(s)` を、ベース速度プロファイルに前方曲率からの制動可能性(`attack_driver.hpp` のブレーキング・コリドー/`braking_corridor_ideal_speed` 相当)を足して、**外側 25 ms ステップごとに 1 回**前処理で作る。既存実装の再利用を優先し、新規ソルバを作らない。
2. 加減速候補を「現在 vx から `v_ref(h)` までの所要 ax 上限/下限の区間を 5 分割」に差し替える。一定 ax という形式は維持しつつ、候補が基準軌道に張り付くため「コーナー向け制動」が候補集合に実在する。
3. `progress_target` の速度基準を base から `v_ref` に変更(R2 解消)。`progress_target_fraction` は v_ref 側へ吸収し、既定 1.0 に近づけて較正する。
4. w_vx の `speed_error` に用いる目標も `v_ref` に統一。

**完了条件(gate):**

- circle r=30(幅 3 m, 初速 6 m/s):角度/トルクともに ラップ ≦ PID+5 %、`max |n| ≦ 0.40 m`。
- `mpc_rollout_progress_deficit` 平均が現行比 1/2 以下、margin 由来の速度損失(report 相当) < 0.6 m/s。
- `ctest -R MPCDriverTest`(35 件)を破らない。API が変わる場合はテストを manifest に記録して移行する。

### P2 — トルクモードの操舵能力モデルを実機に合わせる

**変更:**

1. `predict_steering_actuator_response` に **SAT 反力と T_driver_max 飽和**を入れる。ロールアウトステップごとの操舵レート上限を `|tire_rate| ≦ (T_driver_max + T_SAT_ref/R)/I_tire` 系の量で制約する。`T_SAT_ref` はその時点の a_y 予測とタイヤ Mz 特性から推定し、リテラルオーダー(例: SAT 200 Nm / R=4.5 / I=0.05 で ≈889 rad/s に出るような粗試算)では扱わない。係数は P5 のプラント計測から確定させる。
2. R4 の `compute_steering_authority_scale` 固定 clip を撤去し、(T_max, I, ζ, ω_n, T_SAT推定, vx·κ) から state 依存で決める。決定過程は決論的なので最小の純計算 unit test(deterministic)を付ける。
3. ロールアウトの操舵候補に「この候補を追うのに必要なレートがトルク headroom 内で可能か」のフィルタを付け、飽和を見込むプランは先に落とす(既存 `MPCDriverTest` のサンプリング検証スタイルで数本追加)。

**完了条件(gate):**

- `mpc_v2_fail_run.csv` 相当状況の再現スクリプト(同 params + 同初期条件)で「飽和 → 進捗停止」を再現しない。比較は fail_run の停滞区間の s / target_speed / speed に対し、周回距離・完走・速度誤差で gate 化する。
- circle トルク構成で `steering_lag_ratio` の p95 が現行の 1/2 以下。
- 飽和そのものの完全禁止はノルマにしない(望まないので)。ノルマは「選択したプランが飽和に漕ぎ込んでから再び出てくる」までの時間縮小と、そこから逸脱・停滞しないこと。

### P3 — 反動 cap を前方予防に置き換える

**変更:**

1. PID 側に実在する `line_planner.hpp`(0.25 m キャッシュ)を MPC の FF・cap・進捗計算に接続する: 目標線(中央/プランナーオフセット)の `κ_line(s)` を外側周期で取得し、操舵 FF(via R6 の内補拡張)、速度 cap、進捗目標で一元的に使う。
2. `compute_steering_speed_cap` の現在角ギャップ依存ロジックは削減し、「目標線曲率に対する下限速度 `min(sqrt(a_y_max / |κ|))`」を優先する。`a_y_max` は `driver_benchmark` → `DriverCapabilityProfile`(既存の MPC への override 接続が済み)から取得し、無指定時は保守デフォルト+ログ。
3. `margin cap` は境界外脱出の recovery 用として残してその役割を狭める(release を 1.20 m → 0.25 m 系に緩めて、普段の速度決めには使わない)。

**完了条件(gate):**

- circle `max |n| ≦ 0.20 m`(PID 参照 0.10 m の 2 倍まで)、margin 由来速度損失 < 0.3 m/s。
- `DriverCapabilityProfile` を通した `Attack`/MPC の a_y_max 引用が同一値になっている unit test。

### P4 — 閉路継ぎ目(R7)と manifest 完整化

**変更:** `track_distance.hpp` の最近傍探索に周期 wrap を付ける(既に circle 分析で相対的に軽いが、直進・交差系の創作コースで必ず発火し得る)。検証は `mpc_driver.hpp:511` の signed_wrap と一体で行う。

**完了条件(gate):**

- 合成閉路(自己交差を含む)で継ぎ目を複数回通過する速度での s 不連続 ≦ 0.5 m、heading 投影誤差 < 1° を unit test として固定。
- P0 の manifest 記録がすべての gate run で壊れていないか別のスクリプト/`pytest` smoke で確認(CI 載せ)。

### P5 — 計測と定数の実データへの差し替え(並行可, 低負荷)

- 14DOF プラントでの実効 `a_y_max(v)`、SAT vs a_y 特性、`dFy/dα` 有効値を laptime_sim の診断出力、あるいは `main_driver_benchmark` の拡張ルーチンから計測し、R6 の定数 0.002 と `DriverCapabilityProfile` の μ_y/understeer に差し替える。
- PID / LPV / MPC / Attack が同じデータ(=capability profile)を参照していることを unit test で保証する。

## 4. 実施順(最小実装単位)

| 順 | タスク | 見積 | 依存 |
|---|---|---|---|
| 1 | P0: `scripts/run_mpc_gate_suite.sh` + manifest 保存(circle/skidpad/straight_corner × PID/MPC/MPC_v2) | 0.5 日 | なし |
| 2 | P1: braking-corridor 由来の `v_ref` 前処理(既存実装再利用) | 2 日 | 1 |
| 3 | P1: ax 候補再設計 + `progress_target` v_ref 化 + gate | 1 日 | 2 |
| 4 | P2: SAT/飽和込み操舵予測 + gate(fail_run 反証) | 2 日 | 3(テスト追加は先行可) |
| 5 | P3: line_planner 接続 + `a_y_max` 依存の cap 差し替え | 2〜3 日 | 2 |
| 6 | P4: 閉路 wrap fix + テスト + manifest 完整化 | 1 日 | 1 |
| 7 | P5: 計測 & 定数の実データ差し替え | 1〜2 日 | 随時 |

**合計実質 8〜10 日。** 原則として manifest 先行 → 構造変更 → 計算量最適化は後(2026-09-28 計画 段階3 の措置) → パラメータは信頼できる計測データに合わせて最後。

## 5. 明示的にしないこと(YAGNI)

- グリッド数・ホライゾン長の試行錯誤的調整: 効率性の根拠が不明なまま実補正を積み上げていない(それは学生が空中見えない)。P1/P2 で構造が片付いた後にのみ、2026-09-28 計画の段階3(候補削減・warm start)に進む。
- オフライン NMPC/RTI の導入: 現在の要請はデザインスコープで閉じている。必要になったら 2026-09-28 計画 段階5 参照のうえで WCET・ROI を別に測る。
- OSQP QP 系ドライバーの復活: 本番経路は sampling MPC であり二重保守はしない(DRV-05)。

## 6. 評価ゲート(共通)

全変更は同一 params / track / tire / PINN manifest 上で比較。2026-09-28 計画の評価ゲート表を援用しつつ、本計画では下表の追加判定を付ける。

| 軸 | circle r=30 ゲート | fail_run 相当閉路ゲート |
|---|---|---|
| 完走率 | 1.0(2 連続) | 1.0 |
| ラップ時間 | ≦ PID+5 % | 直前ベースから悪化しない |
| max \|n\| | P1 後 ≦0.40 m → P3 後 ≦0.20 m | コース外移動なし・進捗停滞なし |
| margin 由来速度損失 | P3 後 < 0.3 m/s | — |
| トルク飽和からの脱却 | P2 後: 選択候補の headroom 回復を検出できる | P2 後: 進捗停滞ゼロ |
| wall time / sim s | 悪化 ≦ 20 % | 悪化 ≦ 20 % |

## 7. 関連資料

- `docs/driver_model_improvement_plan_2026-09-28.md` — 上位計画。段階0(基準固定)/段階1(物理・閉路の正確化)は本計画の P0/P4 と一片で扱う。
- `docs/comprehensive_repository_review_2026-09-28.md` — DRV-01〜10。本計画の R7 は該当項との同根、R2/R3 は DRV-04(v2 はアルゴリズムではなく設定バリアント)の反実証データ。
- `docs/driver_control_generalization_plan.md` — mpc_v2 の comparison gate の位置付け。
- `mpc_v2_fail_run.csv`, `mpc_v2_run.csv`, `mpc_v2_report.md`(リポジトリ直下、再現 manifest なし) — 本計画 §1.2 で分析。
