# リポジトリ総合レビュー

- **レビュー日:** 2026-09-28
- **対象:** `sim_overall`
- **レビュー方式:** 静的コード・設定・履歴文書レビュー。物理式は座標系と剛体運動学の符号から検討。実行時挙動や数値性能は既存の記録とコード上の構造を区別して記載。

## 全体所見

本リポジトリには、車両運動、タイヤ、ドライバ、サスペンション・フレーム設計、学習モデルまで一通りの実装がある。一方で、物理式の符号、学習データ分割、閉路トラックの継ぎ目、計算量の大きい予測制御、評価・配布再現性の問題が、結果の信頼性を損ねる可能性がある。

まず対応すべきなのは、(1) 車両ロール・ピッチ・SAT符号を単体の物理ケースで確定する、(2) PINNのモデル選択と正規化をtrain/validation/testの契約に合わせる、(3) 閉路の進捗投影が継ぎ目を越えるようにする、(4) 再現可能な評価条件と実行計測を整える、の4点である。疑義を直ちに実装バグと断定するのではなく、以下の確度表示と再現条件を使って順に切り分ける。

## 対象範囲と限界

確認対象は、C++の14DOF車両・タイヤ・トラック・各ドライバ・シミュレーション入口、PythonのタイヤフィッティングとPINN、MATLABのサスペンション／フレーム最適化、ビルド・依存関係・主要なREADME／計画文書である。行数の多い生成データや全ソース4,000超ファイルを一行ずつ実行確認したという意味での形式的な全数監査ではない。特に全MATLAB経路の数値収束、すべての入力CSVの品質、生成済みレポート／図表の全ページ、実車との物理妥当性は未確認である。

本書はコードを書き換えていない。テスト、ビルド、ベンチマーク、車両走行も実行していない。既存ベンチマークは記録日時・条件を添えて歴史的証拠として扱い、現在の性能保証には使わない。修正は、再現ケースと期待値を先に固定してから行う。

確度は、**高**＝ソースと数式・処理順から直接導ける、**中**＝懸念の根拠はあるが仕様確認や実測が必要、**要確認**＝候補・前提が未確定、を表す。優先度はP1（結果の正当性や主要経路を壊し得る）、P2（重要な性能・一般化・安全性の限界）、P3（保守性・文書・運用負債）である。

## 優先度一覧

| ID | 優先度 | 確度 | 問題 |
|---|---:|---:|---|
| PHY-01 | P1 | 高 | 横力による車体ロールモーメントの符号疑義 |
| PHY-02 | P1 | 高 | 縦力による車体ピッチモーメントの符号疑義 |
| PHY-03 | P1 | 高 | SATがステアを復元でなく増幅する符号疑義 |
| PINN-01 | P1 | 高 | test setでチェックポイントを選択して同じtest setを報告 |
| DRV-01 | P1 | 高 | 閉路トラックの最近傍投影・進捗が周回継ぎ目で循環しない |
| PKG-01 | P1 | 高 | Python editable installがREADME不在で失敗する可能性 |
| DATA-01 | P2 | 高 | TTC raw data・生成物・ビルド資産がignore対象でも追跡されている |
| PHY-04〜08 | P2 | 高〜中 | タイヤMz欠落、姿勢／接地点速度の簡略化、ARB定義、Euler特異点 |
| PINN-02〜10 | P2 | 高〜中 | 正規化漏洩、単位混在、group不足、未接続設定、random split、deployability範囲 |
| DRV-02〜10 | P2 | 高〜中 | Attack/MPC計算量、簡略安全余裕、PINN/TMeasy線形化差、未検証ラップ目標 |
| FRAME-01 | P2 | 高 | GAの評価モードによりFEM計算量が大きく変わる |
| REPRO-01〜03 | P2 | 高 | 依存固定不足、CPU専用最適化既定、CI自動ゲートなし |
| DOC-01〜05 | P2/P3 | 高 | 現行実装とREADME／ロードマップの不一致 |

---

## 1. 物理・車両運動モデル

### PHY-01 — 横力のロールモーメント符号

**対象:** [vehicle_14dof_new.hpp:1299](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1299)、[1443](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1443)、[1448](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1448)

リポジトリの車体系はX前方、Y右、Z下の右手系である。接地点が重心より下にあり、右向きタイヤ横力をFy>0とすると、車体中心から接地点への腕rの下向き成分を正に置いたとき、剛体モーメントのX成分は(r×F)x = -rz Fyとなる。現在の集約処理は横力×正のロール腕を正符号で加算しているため、通常の接地点腕の意味なら期待するロール方向と逆になる疑いが強い。

**影響:** 荷重移動、左右輪荷重、ロール応答、旋回限界、ドライバの軌跡比較に系統誤差が入り得る。

**解決案:** 各輪の実際の接地点・ロール中心に対応する符号付き位置ベクトルを定義し、全モーメントを明示的な外積で集計する。正横力・右旋回の最小解析ケースと、左右反転で符号が反転するテストを追加する。サスペンション由来のモーメントはタイヤ接地点力の腕と別に計上し、二重計上を避ける。

### PHY-02 — 縦力のピッチモーメント符号

**対象:** [vehicle_14dof_new.hpp:1016](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1016)、[1452](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1452)

Z下向きの右手座標系では、重心より下の接地点に前向きFx>0が働く場合、My=(r×F)y=+rz Fxとなる。現在のコードは正の力腕に対して縦力項を減算している。前輪駆動・後輪駆動・制動のどれでも、正負縦力に対するピッチ加速度の期待符号と逆になっている可能性が高い。

**影響:** 加減速時の前後荷重移動、サスペンション荷重、タイヤ縦力余裕に影響する。

**解決案:** 力腕を「符号のない大きさ」として符号反転に使わず、幾何学上のr×Fで計算する。anti-dive／anti-squatはサスペンションリンクが作る別の拘束力・モーメントとして表現し、独立に検証する。一定Fx、左右輪対称の解析テストで加速と制動の応答符号を確定する。

### PHY-03 — セルフアライニングトルクの操舵符号

**対象:** [steering_torque.hpp:41](../cpp/include/fsae/vehicle/steering_torque.hpp#L41)、[vehicle_14dof_new.hpp:1625](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1625)、[tmeasy.hpp:117](../cpp/include/fsae/physics/tire/tmeasy.hpp#L117)

TMeasyのMzは該当箇所でFyに対し負のtrail符号を持つ。右操舵・右向きFyのケースではタイヤモーメントが復元方向になるはずだが、操舵トルク集約側の減算と車両側のMz受け渡しを追うと、右向きFyに対して操舵をさらに増やす可能性がある。

PINNについては出力符号の変換設定も関与するため、すべてのタイヤモデルに共通するバグとまでは確定していない。

**影響:** 操舵減衰・自己整列・定常旋回・トルク操舵の安定性が変わる。

**解決案:** 各タイヤモデルで、正の操舵角、正のFy、正負のMz、操舵トルクの物理方向を表にして変換を一箇所に定義する。モデルごとの単輪ステップテストと、エネルギー／仕事率の符号確認を行い、トルク操舵・角度操舵の両方で確認する。

### PHY-04 — タイヤの自由アライニングモーメントが車体ヨー集約に見えない

**対象:** [vehicle_14dof_new.hpp:1304](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1304)、[1392](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1392)、[1454](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1454)

各輪のtire Mzは得られてキャッシュ・操舵系には渡されるが、車体ヨーモーメントの集約箇所は輪位置とFx/Fyから作る腕モーメントのみで、タイヤの自由モーメントの合計が見当たらない。

**影響:** タイヤの自己整列モーメントが車輪中心・ナックル・サスペンション経由で車体へ伝わる経路を省略している可能性がある。一方、モデルの力・反力の自由度設計によっては別経路で計上済みの可能性があり、二重計上も避ける必要がある。

**解決案:** 各輪のMzが作用する剛体・反力経路を図示し、どの自由度の式に入るか確認する。車体への反力を入れる場合はアクスル・操舵DOFとの作用反作用を整理し、定常円旋回と左右対称テストでヨー応答を確認する。

### PHY-05 — 14DOFの接地点速度・位置が完全な剛体運動学ではない

**対象:** [vehicle_14dof_new.hpp:860](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L860)、[879](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L879)、[1150](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1150)、[1217](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1217)

輪位置のZ成分が0として扱われ、接地点速度では主にヨーレート項が使われる。車体角速度p,qと接地点の上下腕を含む一般式v_contact=v_CG+ω×rにはなっていない。またサスペンション点の変位計算は小角近似を使い、p,qとEuler姿勢率の関係も別途簡略化されている。

**影響:** ロール／ピッチ中のタイヤスリップ角・スリップ比、操舵・サスペンション運動の結合が誤差を持つ。低姿勢角の通常走行では近似が許容できる可能性もある。

**解決案:** モデルの目的範囲を明記する。完全な車体運動学が必要なら、輪ごとの3D位置とω×rから速度を算出する。簡略モデルを維持するなら、姿勢・角速度の有効範囲を定義し、その範囲端でフル式との差を定量化する。

### PHY-06 — ARB剛性パラメータの定義・係数が曖昧

**対象:** [vehicle_params.hpp:51](../cpp/include/fsae/vehicle/vehicle_params.hpp#L51)、[vehicle_14dof_new.hpp:475](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L475)、[1447](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L1447)

パラメータ名・単位はNm/radだが、計算では左右ストローク差をトレッドで割りねじれ相当量を作り、左右に力を配ってから車体ロール項に足している。入力値が棒のねじり率なのか、アクスル等価ロール剛性なのかがコードだけでは確定しない。後者なら2倍相当の可能性がある。

**解決案:** パラメータを「バーねじり剛性」か「アクスル等価ロール剛性」のどちらかに限定して単位と幾何を文書化する。既知のロール角に対する左右輪荷重差の解析テストで係数を確定する。

### PHY-07 — Euler姿勢回転と角速度変換の特異点保護が不一致

**対象:** [vehicle_14dof_new.hpp:882](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L882)、[895](../cpp/include/fsae/vehicle/vehicle_14dof_new.hpp#L895)

回転行列は姿勢thetaをそのまま使う一方、Euler率変換ではthetaを87度にクランプしている。大ロール・横転近傍では座標変換同士の整合が失われる可能性がある。

**影響:** 通常の学生フォーミュラ走行姿勢の外側で姿勢積分が不整合になり得る。通常用途で問題になるかは運用範囲次第。

**解決案:** 対象姿勢を明示して外れ値で停止するか、クォータニオン／回転行列状態を用いる。クランプを残す場合、同一の姿勢状態で力・角速度変換が整合することをテストする。

### PHY-08 — 妥当性確認の要件がコード契約として十分に集約されていない

**対象:** 14DOF・タイヤ・車体・ドライバにまたがるテスト群

符号修正の前提となるゼロスリップ、左右反転、加速／制動、定常円、低速、タイヤ飽和の合否基準が、単一の明確な物理回帰セットとしてレビューからは読み取りにくい。

**解決案:** 数値一致だけのテストに加え、対称性・保存則・符号・単位の解析テストをまとめる。実車照合は別階層で、使ったタイヤデータ・車両パラメータ・トラック・時間刻みを記録する。

---

## 2. ドライバ、トラック、計算量

### DRV-01 — 閉路の最近傍投影が周回継ぎ目で循環しない

**対象:** [track_distance.hpp:76](../cpp/include/fsae/track/track_distance.hpp#L76)、[track_distance.hpp:121](../cpp/include/fsae/track/track_distance.hpp#L121)、[test_mpc.cpp:1128](../cpp/tests/test_mpc.cpp#L1128)

投影処理は探索インデックスと進捗sをトラック端でクランプした後に継ぎ目をまたぐ補正を行うため、閉路として循環できない構造になっている。周回長付近のs refinementも[0, track_length]にクランプされる。MPC/Attackが前回のsをヒントに使うため、1周終端付近で誤った区間に留まるおそれがある。既存の記載テストは30m円を40ステップしか進まず、周回継ぎ目を通過しない。

**影響:** 周回の進捗、前方参照、ラップ計時、予測制御の評価が壊れる可能性がある。

**解決案:** 閉路インデックスをmoduloで扱い、末尾重複点の仕様を決め、sを周期量としてrefineする。自己交差点でのヒント選択規則も定義する。正・逆方向に複数周し、継ぎ目の前後で位置・進捗・参照姿勢が連続であることを確認する。

### DRV-02 — MPCの全候補予測と各ステップの投影が重い

**対象:** [mpc_driver.hpp:228](../cpp/include/fsae/driver/mpc_driver.hpp#L228)、[mpc_driver.hpp:480](../cpp/include/fsae/driver/mpc_driver.hpp#L480)、[track_distance.hpp:76](../cpp/include/fsae/track/track_distance.hpp#L76)、[main_laptime_sim.cpp:2124](../cpp/src/main_laptime_sim.cpp#L2124)

標準構成では7操舵×5加減速の候補を有限ホライズンで全評価し、Autocross設定のホライズン60なら更新当たり約2,100予測ステップになる。制御更新は25ms周期。各ステップで最近傍探索と予見速度の走査を行い、候補間にも同じトラック幾何を繰り返し処理する。これはコードから数えた構造的コストであり、現在の実測時間ではない。

**解決案:** 先にステップ数、投影回数、速度上限走査回数、候補の重複数を計測する。まず経路・速度プロファイルの事前計算、周期補間、同一候補の除去、rollout中の参照再利用など判断を変えない最適化を行う。候補削減は同一条件のラップ・安全性比較を通してから導入する。

### DRV-03 — Attackは予測候補と各候補内の参照処理を重ねている

**対象:** [attack_driver.hpp:1156](../cpp/include/fsae/driver/attack_driver.hpp#L1156)、[1307](../cpp/include/fsae/driver/attack_driver.hpp#L1307)、[1218](../cpp/include/fsae/driver/attack_driver.hpp#L1218)、[1354](../cpp/include/fsae/driver/attack_driver.hpp#L1354)

通常は7操舵プリミティブ×4速度倍率=28ロールアウト候補を持ち、各ステップで進捗を投影し、最大32m程度先まで速度目標を探索する。トルク操舵ではホライズンが60になる経路もある。候補前後の投影・速度参照計算が同じ情報に対し反復される。

**解決案:** MPC同様にプロファイルと幾何を事前計算し、rollout状態ごとの投影は必要回数に絞る。過去の候補時間分布が古いものかを再計測し、中央値だけでなくp95/p99/maxも残す。

### DRV-04 — MPC v2は新しい最適化方式ではなく既存サンプリング方式の設定ラッパー

**対象:** [mpc_driver_v2.hpp:14](../cpp/include/fsae/driver/mpc_driver_v2.hpp#L14)、[64](../cpp/include/fsae/driver/mpc_driver_v2.hpp#L64)

現実装は既存ドライバのパラメータ差分を包む構造で、独立した数理最適化器ではない。ユーザー向け名称だけではアルゴリズム上の差を誤解しやすい。

**解決案:** 文書・CLI説明に「設定バリアント」と明記し、比較の際は変わるパラメータと同一の部分を列挙する。別アルゴリズム化が必要なら別ID・別評価を設ける。

### DRV-05 — OSQP系MPCProblemと本番ドライバの経路が分かれている

**対象:** [mpc_problem.hpp](../cpp/include/fsae/control/mpc_problem.hpp)、[mpc_driver.hpp:5](../cpp/include/fsae/driver/mpc_driver.hpp#L5)

QP形式の問題定義／OSQP接続がテスト・ヘッダー側に残る一方、実運用のMPCドライバは有限候補グリッドのrollout方式である。二つの経路のどちらが本命かが読者に分かりにくく、依存ライブラリや実装が陳腐化する可能性がある。

**解決案:** 本番未使用であれば実験用として明記し、依存性と保守範囲を整理する。使用するなら本番入口から評価経路まで一貫させ、同一条件で比較する。

### DRV-06 — 安全フィルタの余裕はCG点の境界距離で、車体包絡や摩擦制約保証ではない

**対象:** [analytic_safety_filter.hpp:5](../cpp/include/fsae/driver/analytic_safety_filter.hpp#L5)、[37](../cpp/include/fsae/driver/analytic_safety_filter.hpp#L37)、[mpc_driver.hpp:775](../cpp/include/fsae/driver/mpc_driver.hpp#L775)

境界余裕はCGから境界の距離を主に扱うスカラー近似である。車体幅・ホイールベースの包絡、タイヤ合成力・摩擦円、曲率を考えたFrenet横変位の変化率までを保証する正式なCBFとしては定義されていない。ヘッダー自体も形式的CBF／摩擦円制約ではない範囲を示している。

**解決案:** 名称とドキュメントで助言的governorの範囲を保つ。車体外形を含む境界制約、タイヤ合成力制約、緊急制動余裕が必要なら別の制約モデルとして定式化し、外側走行を含むテストで評価する。

### DRV-07 — LPV-LQRの線形化タイヤと真値プラントのタイヤが異なる

**対象:** [bicycle_model.hpp:68](../cpp/include/fsae/vehicle/bicycle_model.hpp#L68)、[lpv_lqr_driver_experiment_2026-07-19.md:155](../docs/lpv_lqr_driver_experiment_2026-07-19.md#L155)

LPV用のCf/CrはTMeasyパラメータから作られる一方、評価プラントはPINNタイヤを使う構成がある。過去の限定条件でヨーゲイン差が小さいという記録はあるが、荷重・スリップ・温度・組合せスリップ全域で同等とは示していない。

**解決案:** 使用プロファイルごとのPINN局所勾配dFy/dalphaを比較し、適用領域と逸脱時の低下挙動を記録する。速度・荷重・スリップ・タイヤ状態ごとの誤差表を残し、必要なら実プラント応答から同定する。

### DRV-08 — 66秒ラップ目標に再現可能な根拠が不足

**対象:** [driver_model_zero_base_assessment_2026-07-18.md:105](../docs/driver_model_zero_base_assessment_2026-07-18.md#L105)

過去の文書自身が、66.0秒目標を実測ラップのハッシュ、トラック整列、車両設定、計時規約に結び付けていないと記録している。この値は現状、同一条件で再現されたベンチマーク目標ではない。

**解決案:** 実測・設計目標・シミュレーション目標を分ける。目標値に使うなら、ソース走行、トラック変換、始終点、定常状態の除外、車両とタイヤの設定を含むマニフェストを作る。

### DRV-09 — 高速評価車両モデルではトルク操舵を評価できない

**対象:** [main_laptime_sim.cpp:1023](../cpp/src/main_laptime_sim.cpp#L1023)、[fast_lap_vehicle.hpp:14](../cpp/include/fsae/vehicle/fast_lap_vehicle.hpp#L14)

軽量評価経路は6DOF/TMeasy近似を用い、トルク操舵を明示的に拒否する。操舵自由度・SATとの結合はフル車両側にあるため、軽量モデルのみでトルク操舵ドライバを最終評価できない。

**解決案:** まず評価器のサポート範囲を明示する。トルク操舵の設計探索に軽量モデルを使う前に、操舵アクチュエータ・SAT・タイヤモデルを含む予測器の誤差をフル14DOFに対して校正する。勝者候補はフルプラントで必ず再評価する。

### DRV-10 — ドライバ更新周期・シミュレーション時間・実時間制約が混同されやすい

**対象:** [main_laptime_sim.cpp:2112](../cpp/src/main_laptime_sim.cpp#L2112)、[2124](../cpp/src/main_laptime_sim.cpp#L2124)

PIDは1ms刻み、RL更新は20ms、予測系は25ms周期の構成がある。シミュレーションが速く完了することと、実車で締切り内に制御できることは別要件である。実機CPU・許容制御周期・最悪応答時間がなければリアルタイム性は判定できない。

**解決案:** offline throughput（実時間／シミュレーション時間）とonline deadline（1回の更新のWCET）を別々に報告する。実時間制御を要する場合のみ、対象CPUと周期を要件にしてp99/maxでゲートする。

---

## 3. タイヤフィッティング・PINN

### PINN-01 — test setでチェックポイントを選び、そのtest set性能を報告

**対象:** [main_pinn_train.py:520](../tire_pinn/main_pinn_train.py#L520)、[550](../tire_pinn/main_pinn_train.py#L550)、[583](../tire_pinn/main_pinn_train.py#L583)

best R²、best model、final modelをtest loaderで評価し、その結果を使って候補モデルを選んだ後に同じtest setの性能として記録している。testは最終汎化評価に独立に保つ必要があるため、報告値が選択に過適合する。

**解決案:** checkpointとハイパーパラメータをvalidationだけで選ぶ。選択を凍結してからtestを一度だけ評価し、選択前のtest結果を最終性能として扱わない。既存の値は比較参考へ格下げする。

### PINN-02 — 正規化統計が分割前の全データから算出される

**対象:** [ttc_loader.py:103](../tire_pinn/data/ttc_loader.py#L103)、[274](../tire_pinn/data/ttc_loader.py#L274)、[main_pinn_train.py:450](../tire_pinn/main_pinn_train.py#L450)

平均・標準偏差を全DataFrameから算出してからデータを分割しているため、validation/test分布の情報が入力・出力の正規化に混入する。また一部マスクされた出力を含む統計の取り扱いも明確でない。

**解決案:** source/run groupを先にtrain/validation/testへ分ける。標準化統計はtrainのみ、出力ごとの有効ラベルで計算し、保存モデルへ統計とsplit IDを記録する。

### PINN-03 — 力NとモーメントNmに同一の閾値を使う

**対象:** [physics_losses.py:23](../tire_pinn/losses/physics_losses.py#L23)、[67](../tire_pinn/losses/physics_losses.py#L67)、[physics_tests.py:144](../tire_pinn/validation/physics_tests.py#L144)

ゼロスリップ残差や検査ではFx・Fy（N）とMz（Nm）を同じforce_thresholdで扱う箇所がある。単位の異なる出力へ同一の絶対しきい値を適用すると、物理的意味が一致しない。設定名が最大許容誤差に読める一方、損失内では除算スケールとして使われる箇所もある。

**解決案:** 力とモーメントで別のスケール・閾値・単位・許容誤差を定義する。損失の無次元化係数と合否ゲートは別の設定・レポートにする。

### PINN-04 — source groupが少ない場合validationとtestが同じgroupになる

**対象:** [ttc_loader.py:380](../tire_pinn/data/ttc_loader.py#L380)、[458](../tire_pinn/data/ttc_loader.py#L458)

sourceが2つのとき一方をvalidationとtestの双方へ置き、3つのときも最後のgroupが両方に使われる分岐がある。独立validation・testとして解釈できない。

**解決案:** 独立group数が足りなければ分割性能を定義できないと明示する。LOCO/CVまたはnested group validationを用いるか、データ追加を要求する。重複groupで作った既存結果はその事実を明示する。

### PINN-05 — YAMLの物理損失設定の一部がtrainerへ接続されていない

**対象:** [pinn_config.yaml:59](../tire_pinn/config/pinn_config.yaml#L59)、[trainer.py:98](../tire_pinn/training/trainer.py#L98)、[523](../tire_pinn/training/trainer.py#L523)

YAMLにcollocation数、しきい値、摩擦係数上限などの設定があるが、trainer側ではデフォルト引数を直接用いたりcollocation数を固定したりする箇所があり、設定変更が実際の損失・サンプラーへ反映されないケースがある。

**解決案:** 設定値を各loss/samplerの引数へ明示的に接続し、起動時に採用値をログへ出す。未対応設定を黙って無視せずエラーにする設定スキーマ検査を追加する。

### PINN-06 — 一部プロファイルがrandom point splitを使う

**対象:** [pinn_config.yaml:499](../tire_pinn/config/pinn_config.yaml#L499)、[593](../tire_pinn/config/pinn_config.yaml#L593)

ランダム分割は同じ測定run由来の隣接・類似点がtrainとtestへ混ざり、測定条件またはタイヤ個体をまたぐ一般化を示さない可能性がある。

**解決案:** random split結果は同一分布内補間評価とラベル付けする。run・試験セッション・タイヤ個体など、使いたい汎化単位をholdoutにした評価を別に設ける。

### PINN-07 — deployableゲートが利用用途の要求出力を全て見ていない

**対象:** [main_pinn_train.py:130](../tire_pinn/main_pinn_train.py#L130)、[tire_catalog.py](../tire_pinn/data/tire_catalog.py)

モデル採用判定は主にDriveBrake/Corneringの完全性、物理ゲート、Fx/Fyの最低R²を見る。combined-slipやMzの精度が必須条件になっていない構成でも、車両側でMzが操舵SATに使われる。

**解決案:** deployableは一律の真偽値でなく用途範囲に結び付ける。合成スリップを使う走行条件なら該当するholdout測定を必須にし、SATを有効にする場合はMz符号・大きさ・操舵安定性の評価を必須にする。

### PINN-08 — 合成物理テストが実測された共同分布の検証ではない

**対象:** [physics_tests.py:237](../tire_pinn/validation/physics_tests.py#L237)

SR、SA、荷重、キャンバー、TSTCを独立一様サンプルする摩擦円チェックは、広い入力範囲の合成probeとして有用だが、実際のタイヤ試験や車両で到達する相関した入力分布を検証しない。

**解決案:** 合成物理テストと独立に、holdout測定run、物理的に到達する相関軌跡、combined-slip試験を使った検証を設ける。各ゲートが証明する範囲をレポートで区別する。

### PINN-09 — 入力変数と利用範囲に未観測要因がある

現行入力はSR、SA、Fz、IA、TSTCを中心としており、圧力・摩耗・温度・タイヤ個体差などが欠けている場合、学習領域外でこれら要因の変化を表せない。

**解決案:** 実験・利用範囲を明示し、未入力要因を固定条件として記録する。広い範囲へ使う主張をするなら、必要な変数・測定を加えて個体／条件holdoutを実施する。

### PINN-10 — TMeasy実装名から標準モデル忠実度を誤認し得る

**対象:** [tmeasy.hpp:72](../cpp/include/fsae/physics/tire/tmeasy.hpp#L72)、[79](../cpp/include/fsae/physics/tire/tmeasy.hpp#L79)、[117](../cpp/include/fsae/physics/tire/tmeasy.hpp#L117)

実装には単純な荷重依存μ、飽和形状、trail近似が含まれる。v_effが計算される一方で緩和項は別速度を使うなど、近似の適用範囲をコードだけから標準TMeasy全仕様と同等とは判断できない。

**解決案:** 実装が採用しているTMeasyの式と省略項を説明する。低速、ゼロスリップ、飽和、荷重範囲、Mz、緩和長の測定曲線で適用範囲を確認し、名称・文書の忠実度を合わせる。

---

## 4. 最適化、依存性、再現性、データ管理

### PKG-01 — Pythonパッケージのeditable installがREADME欠落で失敗する可能性

**対象:** [pyproject.toml:5](../python/pyproject.toml#L5)、[setup_env.sh:118](../setup_env.sh#L118)、[setup_env.bat:36](../setup_env.bat#L36)

python/pyproject.tomlはREADME.mdを指定しているが、python/README.mdが見つからず、環境構築スクリプトはpythonディレクトリでeditable installを実行する。実際のpip installは試していないため、失敗可能性として記録する。

**解決案:** READMEの正しいファイルを指定するかpython/README.mdを加え、クリーン環境でeditable installを確認する。OSごとのセットアップスクリプトとCIを同じ入口で検証する。

### DATA-01 — ignore対象でも大容量・生成・生データがGit追跡対象

**対象:** [.gitignore](../.gitignore)

調査時点でignore条件に一致する追跡ファイルが約1,456件あり、その多くはvcpkg_installed、build、出力ファイル、TTCのraw .datなど。data/tiredataは約1.6GBの追跡資産、root output以下は作業ツリー上で約31GBだった（規模は調査時点）。ignoreは既に追跡された履歴を除外しない。リポジトリは2026-09-28時点でprivateであることを外部情報で確認したが、privateであってもraw測定データの配布・履歴保持方針は明示が必要。

**影響:** clone・CI・バックアップの肥大化、機密・ライセンス条件の誤配布、成果物とソース変更の混在につながる。

**解決案:** 今回のレビューで削除や履歴書換えは行わない。将来の整理ではソース、再生成可能物、ライセンス制約のある測定データを分類し、許可リスト・保管場所・履歴移行計画を別途決める。公開用リポジトリは元履歴と分ける。

### REPRO-01 — 依存バージョンを再現する固定点が不足

**対象:** [vcpkg.json](../vcpkg.json)、[python/pyproject.toml](../python/pyproject.toml)、[tire_pinn/requirements.txt](../tire_pinn/requirements.txt)、[Dockerfile](../Dockerfile)

vcpkg baselineが固定されず、Python依存も上限／hashがない範囲がある。Dockerではベース環境やapt/pip依存が浮動する。年月をまたいで同じ学習・ビルドを再現しにくい。

**解決案:** vcpkg baseline、Python lock＋hash、コンテナベースdigestとOSパッケージのスナップショットを採用し、使用コンパイラ・OS・CPU・数値ライブラリを成果物に記録する。ツール導入を増やす前に既存の標準ロック手段を優先する。

### REPRO-02 — native CPU最適化が既定で他環境の実行性を損なう

**対象:** [CMakeLists.txt:10](../CMakeLists.txt#L10)、[20](../CMakeLists.txt#L20)

ビルド設定はauto profileとnative向けコンパイルオプションを自動で有効にする構造。作成CPU以外への配布やCI再利用で未対応命令が含まれる可能性がある。

**解決案:** 配布・CIビルドはポータブルを既定とし、native最適化は明示的なローカルbenchmark optionにする。最小サポートCPUと実行可能範囲を文書化する。

### FRAME-01 — フレームGAの評価ケース数によりFEMコストが大幅に変わる

**対象:** [frameObjectiveFunction.m:219](../frame_optimize/matlab/frameObjectiveFunction.m#L219)、[405](../frame_optimize/matlab/frameObjectiveFunction.m#L405)、[run_frame_v2_targeted_ga_budget.m:5](../frame_optimize/examples/run_frame_v2_targeted_ga_budget.m#L5)

V2目的関数は既定で全荷重ケースを評価し、一候補当たり5ケースのFEM solveとなる構成がある。一方、ターゲットGAヘルパーは予算上torsion_onlyを選べる。15,000評価を全ケースで実行すると単純積で最大約75,000 solveになる。

**解決案:** 高価な全ケース最適化の前に、候補生成・安価なスクリーニング・最終全ケース検証を分離する。torsion_onlyの候補を最終解として扱わない。FEM solve数、収束失敗数、全ケース合格率を計測する。

### REPRO-03 — 自動回帰ゲートが見当たらない

**対象:** [.github](../.github)

GitHub Actionsはレビュー範囲でdependabot設定のみを確認し、configure/build/ctest、Python packaging、主要データ・文書整合の自動ゲートが見当たらない。

**影響:** 符号・依存・文書の変更がユーザー環境で初めて壊れるリスクが高い。

**解決案:** まず小さなCIでCMake configure/build/ctest、Python package build/import、主要単体検査を分けて追加する。重いMATLAB、学習、全ラップは別の手動または定期ゲートにし、CI時間と依存を増やし過ぎない。

### OPT-01 — A-arm最適化で数値ソルバを大量に反復する

**対象:** [main_a_arm_optimizer.m:75](../suspension_kinematics_design/main_a_arm_optimizer.m#L75)、[205](../suspension_kinematics_design/main_a_arm_optimizer.m#L205)、[evaluate_double_wishbone_3d.m:148](../suspension_kinematics_design/evaluate_double_wishbone_3d.m#L148)

SQP/MultiStartを逐次実行し、目的関数評価の中でtravel点ごとにfsolveを使う経路がある。数値勾配も含めた実測コストは未確認。

**解決案:** 先に評価回数・fsolve反復・失敗率をprofileする。近接travel点のwarm start、対象指標に十分なサンプル数、解析微分の費用対効果を調べる。並列化は再現性とライセンスを確認してから選ぶ。

### OPT-02 — MATLAB統合入口のヘッダー既定値が実際の既定値と不一致

**対象:** [run_full_pipeline.m:16](../matlab/run_full_pipeline.m#L16)、[49](../matlab/run_full_pipeline.m#L49)

冒頭コメントはn_iter 30・sim_time 40・初期速度8を示す一方、実行値は60・180・15である。実行経路の段階的上書きの説明も読者を混乱させる。

**解決案:** 実値に合わせてヘッダーを更新し、イベント別上書きは別表にする。設定値の単一情報源から説明を出せる形にする。

---

## 5. ドキュメント・操作契約

### DOC-01 — 現行ドライバ一覧と現在システムマップが不一致

**対象:** [current_system_map.md:11](../docs/current_system_map.md#L11)、[main_laptime_sim.cpp](../cpp/src/main_laptime_sim.cpp)

システムマップの説明はPID/RL中心だが、CLIにはpid、lpv_lqr、mpc、mpc_v2、attack、rlが存在する。

**解決案:** 各方式の数理モデル、更新周期、予測器、実験／本番状態を一覧化し、CLIの選択肢から更新する。

### DOC-02 — ロードマップ内のアーキテクチャ状態が現状と食い違う

**対象:** [future_roadmap.md:9](../docs/future_roadmap.md#L9)、[37](../docs/future_roadmap.md#L37)、[76](../docs/future_roadmap.md#L76)

14DOF統合を将来項目として記載する箇所、削除済みsuspension_base_geometry.jsonへの参照、PID/RLを主方式とする説明が、現行実装と合わない。

**解決案:** ロードマップを「完了」「進行中」「未着手」で再構成し、現行のシステムマップを参照元にする。

### DOC-03 — POU READMEが実装済み機能を未実装扱いする

**対象:** [pou_hardpoint_design/README.md:239](../matlab/pou_hardpoint_design/README.md#L239)、[run_all_wheels.m:147](../matlab/pou_hardpoint_design/run_all_wheels.m#L147)、[run_full_pipeline.m:319](../matlab/run_full_pipeline.m#L319)

READMEがCSV/JSON出力、複数輪最適化、run_full_pipeline統合を将来機能と記載する一方、現行コードには出力とopt-in統合経路がある。ただし既定で全輪実行ではないため、対応範囲の区別は必要。

**解決案:** 「実装済み・任意・未実装」を分け、前輪のみなどの既定範囲も書く。

### DOC-04 — Pythonの文書リンクと依存導入案内が存在しないファイルを指す

**対象:** [docs/source_reference/python/README.md:6](../docs/source_reference/python/README.md#L6)、[tire_fitting/README.md:26](../tire_fitting/README.md#L26)

pyproject.toml.mdへのリンク先がなく、tire_fittingの手順がルートにないrequirements.txtを参照している。

**解決案:** 実在するpyproject.tomlや適切なrequirements/lockへ修正し、相対リンクチェックを自動化する。

### DOC-05 — PINN実装状態文書がネットワーク・損失構成の現在値と古い

**対象:** [IMPLEMENTATION_STATUS.md:5](../tire_pinn/IMPLEMENTATION_STATUS.md#L5)、[pinn_config.yaml](../tire_pinn/config/pinn_config.yaml)

2026-02-25時点の層数・モデル構造・損失説明が、現行のmulti-head設定・カスタム構成と一致しない可能性がある。

**解決案:** 実行設定から現行のモデル概要と接続済み損失を説明するよう更新する。再学習結果を反映する文書には日付・config hash・データsplitを明記する。

---

## 対応の推奨順

1. PHY-01〜03を、モデル別の最小解析ケースで符号確定する。直ちに符号を反転する前に期待方向と作用反作用を固定する。
2. PINN-01〜05を修正し、既存の評価値を新しい独立holdoutと混同しない形に整理する。
3. DRV-01の周回継ぎ目問題を直し、周回横断テストで確認する。
4. driver・FEM・A-armの構造的コストを同一環境でprofileし、判断を変えない最適化から着手する。別紙の[ドライバモデル改善計画](./driver_model_improvement_plan_2026-09-28.md)に段階と評価ゲートを示した。
5. PKG-01・REPRO-01〜03の小さな自動ゲートを整える。
6. 仕様と実装が一致してからDOC-01〜05を更新する。
