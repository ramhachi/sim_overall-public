# フレーム最適化システム 現状問題点調査

## 目的

現行のフレーム最適化について、**いまの実装がどこで詰まりやすいか / どこが誤解を招くか / どこが下流につながっていないか**を、コードと文書の実態ベースで整理する。  
本書は新機能提案ではなく、**現状の問題点監査**である。

## 結論

現行のフレーム最適化は、**単体デモとしては動く前提がある一方で、標準運用フローへの接続・自動化・文書整合性が弱い**。特に大きい問題は次の 5 点。

| 重要度 | 問題 | 影響 |
| --- | --- | --- |
| 高 | 実行フローが対話依存で自動化しにくい | EasyStart からは回せても、統合パイプラインや再現実験に乗せにくい |
| 高 | 保存先と下流読込の導線が弱い | フレーム最適化結果を出しても、次のシミュレーションに自然接続されない |
| 高 | 旧 API / 旧テスト / 旧デモが残り、入口が二重化している | 保守時に「どれが正か」が分かりにくく、壊れた経路も混在する |
| 中 | 規則チェック・バリデーションの実装と運用が乖離している | README の期待ほど一貫して適合性保証されない |
| 中 | artifact には詳細が保存されるが、下流で使う主経路は限定的 | `params.frame` を保存しても、標準導線での実利が薄い |

## V2 実装後の更新

この監査で挙げた問題のうち、V2 改良で次を実装済みにした。

- `frame_design_schema_v2()` と `decode_frame_design_vector()` により、V2 の設計変数名・単位・上下限・初期値・整数/連続を一元管理する
- V1 の 18 変数は `frame_design/v1` として互換維持し、V2 は `frame_design/v2` として追加する
- `generate_parametric_nodes_v2()` / `generate_adaptive_topology_v2()` により、FBH / Front Hoop / Main Hoop / Shoulder Bar / Main Hoop Brace / リア終端台形を設計変数化する
- V2 の group 名は `front_hoop`, `main_hoop`, `side_impact`, `front_bulkhead`, `fbh_support`, `front_susp_mount`, `mh_bracing`, `mh_brace_support`, `shoulder_bar`, `component_protection`, `diff_mount`, `rear_box`, `secondary_brace` に分け、`structural` / `shoulder` の曖昧語彙を避ける
- V2 は diff mount input がない場合に fail-fast し、台形終端より後方だけをリア自由領域として扱う
- V2 は FEM 前の cheap check で MH 幅、FH/MH 傾き、Side Impact 高さ帯、MHB 取付高さ/角度、孤立ノード、リア最大管長を棄却する
- V2 は torsion だけでなく lateral / braking / rollover / diff reaction を生成し、目的関数と artifact にケース別結果を残す
- `params.frame.schema_version` は V2 経路で `frame_result/v2` となり、設計 schema、全設計変数、optimizer mode / seed、solver backend、FEM lifecycle、目的関数係数、荷重ケース結果、規定レポートを保存する
- `run_frame_optimizer_mode.m` と `run_frame_optimizer_comparison.m` により、GA 固定ではなく `mixed_ga`, `lhs_local`, `nsga2_like`, `local_refine` を同一評価予算で比較できる入口を追加した
- 2026-05-10 の追修正で、MEX backend 優先、TCP FEM lifecycle mode、hardpoint cache、MATLAB `ga` 用の有限 rejection penalty、feasible-neighborhood initial population、V2 目的関数係数の再スケールを入れた

残る限界は、応力 / 要素内力が C++ FEM からまだ返らないこと、driver/cockpit template がまだ proxy 判定であること、`nsga2_like` が完全な Pareto rank 実装ではなく scalarized 比較 scaffold であること、下流 14DOF 側が主に `K_chassis` を読む構造のままであること。

## 2026-05-10 V2 15000 評価ベンチマーク

目標ねじり剛性 7000 Nm/deg、重量目標 25 kg、MATLAB `ga`、population 100、seed 7000 で V2 を 15000 評価規模で実行した。MATLAB `ga` は elite 個体を再評価しないため、世代数は 157 に補正し、GA ログ上 15020 評価、最終フルケース再評価込みで `actual_funccount=15021` となった。

結果:

- 実行時間: 322.8 s、0.0215 s/eval
- 最終フレーム: 65 nodes、112 elements
- 質量: 66.96 kg
- ねじり剛性: 7548 Nm/deg
- CG Z: 0.377 m
- 規定チェック: pass、compliance penalty 0
- 荷重ケース: torsion / lateral_bending / longitudinal_bending / rollover / diff_reaction はすべて FEM 成功
- 最大変位: lateral 0.02488 m、rollover 0.02422 m で、現行上限 0.025 m に近い

同じ V2 の full load-case 目的関数ベンチマークでは 0.2347 s/eval だった。したがって、V1 制作時に見ていた 2-5 ms/eval 級の説明と比べて V2 full が 50-100 倍遅く見える主因は、単純な GA 実装差ではなく次の複合要因である。

- V1 で想定していた高速 MEX / 単一 torsion 評価に対し、V2 full は TCP/Docker FEM を複数荷重ケースで呼ぶ
- V2 full は torsion / lateral / braking / rollover / diff reaction の最大 5 FEM solve を 1 個体ごとに実行する
- topology が大きくなり、典型個体で 65-69 nodes、112-124 elements を扱う
- FEM 前後で geometry cheap check、規定 group check、exclusion、diff/rear free region、artifact 保存用メタデータ生成が増えた
- ランダム GA の多くの個体が制約違反になり、Mean が Inf の世代が長く続くため、評価予算の探索効率が悪い
- console output や Docker/TCP round-trip が小規模 FEM では相対的に重く見える

今回の `run_frame_v2_targeted_ga_budget.m` は、探索中を `torsion_only` にして最後だけ全ケース再評価することで 0.0215 s/eval まで戻した。ただし、これは全荷重ケース同時最適化ではなく、横曲げ / rollover 変位は最終確認でぎりぎり通っている状態である。次の改善は、full 評価を毎世代全個体へ掛けるのではなく、cheap surrogate、段階評価、feasible-preserving encoding、上位個体のみ full case 評価を組み合わせる方向が妥当である。

## 2026-05-10 V2 安定化後の短時間実験

上記ベンチマーク後、重量・速度・GA 破綻に対して初回修正を入れた。代表 run は `output/runtime/frame_v2_ga_300_mex_final`。

結果:

- 実行時間: 61.2 s、0.1980 s/eval
- actual funccount: 309
- finite score rate: 100%
- true valid individual rate: 62.5%
- 最終フレーム: 65 nodes、97 elements
- 質量: 45.06 kg
- ねじり剛性: 4909 Nm/deg
- 総管長: 40.62 m
- 規定チェック: pass、compliance penalty 0
- MEX torsion median: 0.001432 s/eval

これにより、Docker/TCP lifecycle と `Inf` population 統計の問題は調査 run では大きく改善した。一方で 25 kg / 7000 Nm/deg 目標にはまだ未達であり、現在の mandatory topology では総管長が 40 m 級に残る。次は GA 評価数を増やす前に、現 topology の最軽量 compliant pipe 割当と、Side Impact / FBH support / MH brace support / rear box の topology switch 化で、そもそもの質量下限を確認する必要がある。

## 2026-05-11 規定最小トポロジーと規定チェッカー監査

添付 rulebook `FSAE_Rules_2026_V1.md` と同階層画像を再確認した結果、チェッカー/モデル側に次の重要点が見つかった。

- `frame_optimize/data/regulation_params.json` の `fbh_support.required_size` が Size B になっていたが、F.3.2.1 表では Front Bulkhead Support は Size C であるため修正した
- Size D は A/B/C の単純な上下関係ではなく、Bent/Multi Upper Side Impact 用に `D_min=35 mm` を持つ特殊クラスであるため、`checkFrameCompliance.m` は class rank ではなく `I/A/D/t` の実寸閾値で判定するよう修正した
- 現行の包括 `checkFrameCompliance.m` はサイズ/材料/最低限 group 存在の確認が中心で、F.6.2 FBH support の片側3本、F.6.3 Front Hoop Bracing、F.6.4 Side Impact の上下/対角、F.6.6 MHB support の片側2本、driver/cockpit template までは完全には保証していない
- V2 の cheap check は MH幅、FH/MH傾き、Side Impact高さ帯、MHB高さ/角度などを見るが、現V2 node set では Upper Side Impact を FH/MH の上部ノードに接続する近似が混ざっており、F.6.4 の 290-345 mm 帯を管全体で厳密に満たすモデルではない

同じ V2 geometry artifact に対して、規定管に近い sparse topology を実験用に生成した。入口は `experiment_frame_v2_mass_cap_stiffness(..., 'TopologyMode','regulation_minimal')`。

| Topology | Nodes | Elements | Lightest mass kg | K torsion Nm/deg | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| Saved fixed topology | 65 | 97 | 31.03 | 3479 | 28 kg に 3.03 kg 超過 |
| Regulation-minimum + diff mounts | 61 | 82 | 27.23 | 3357 | diff reaction まで FEM 成功 |
| Regulation-minimum + suspension only | 57 | 74 | 24.28 | 3357 | diff node は剪定、diff reaction は未評価 |

この結果から、質量問題の主因はパイプ選択だけではなく、現在の固定 topology に含まれる非規定 rear/free-region 管である可能性が高い。次は sparse topology を正式な初期値/seed に昇格する前に、Side Impact と Front Hoop Bracing / FBH Support の規定幾何をノードレベルで作り直す必要がある。

## 1. 実行フローが対話依存で、自動化しにくい

現行の主入口 `frame_optimize/examples/run_frame_optimization.m` は、最適化の途中と末尾に複数の対話操作を要求する。

- 目標重量を `input(...)` で手入力させる。`frame_optimize/examples/run_frame_optimization.m:115-129`
- SolidWorks マクロ生成可否も `input(...)` で都度判断させる。`frame_optimize/examples/run_frame_optimization.m:337-350`
- 最後は `uiwait(gcf)` で GUI を閉じるまで処理が戻らない。`frame_optimize/examples/run_frame_optimization.m:462-467`
- FEM サーバ起動 / 停止もスクリプト側で直接管理している。`frame_optimize/examples/run_frame_optimization.m:274-276,462-464`

さらに EasyStart 側も、関数呼び出しではなく **base workspace へ変数を注入して `evalin('base', 'run(...)')` する**構造になっている。`EasyStart.m:1027-1035`  
このため、バッチ実行・CI・統合パイプライン・再現性確認のどれに対しても扱いが重い。

## 2. 保存はできるが、標準パイプラインへの戻り道が弱い

フレーム最適化の出力は、`vehicle_params_file` が与えられないと `output/runtime/frame_optimized_vehicle_artifact.json` に保存される。`frame_optimize/examples/run_frame_optimization.m:49-60,372-385`

一方、`matlab/run_full_pipeline.m` の標準導線は `optimized_autocross.json` または `optimized_<track>.json`、もしくは `opt.custom_params_file` を起点にする。`matlab/run_full_pipeline.m:208-280`  
つまり、**フレーム最適化の結果を出しただけでは、次回の標準シミュレーションが自動でそれを拾うわけではない**。

このズレは文書とも衝突している。`docs/current_system_map.md` では「フレーム最適化の詳細結果は同じ vehicle artifact JSON に保存」と説明しているが、`optimized_*.json` を使う標準導線と `output/runtime/frame_optimized_vehicle_artifact.json` は別導線になりうる。`docs/current_system_map.md:96-102`

EasyStart 側も、完了後は「完了しました」とだけ表示してメニューへ戻るため、**次にどの JSON を使えば結果を反映できるか**の案内がない。`EasyStart.m:1033-1040`

## 3. 入口が二重化しており、旧経路が壊れている

システムマップ上でも、フレーム最適化の入口として `run_frame_optimization.m` と `runFrameOptimization.m` の両方が並記されている。`docs/current_system_map.md:13-15`

しかし両者は同じ状態ではない。

- 現行主入口は `run_frame_optimization.m` で、ここでは  
  `frameObjectiveFunction(x, [], [], [], config, catalog, materials, reg_params)` を呼ぶ。`frame_optimize/examples/run_frame_optimization.m:281-283`
- 一方、旧風の `runFrameOptimization.m` は  
  `frameObjectiveFunction(x, nodes, elements_geom, bcs, loads, config, catalog, materials, reg_params)` を呼ぶ。`frame_optimize/matlab/runFrameOptimization.m:39`
- ところが `frameObjectiveFunction` の定義は  
  `function [obj_val, results] = frameObjectiveFunction(x, nodes_base, bcs, loads, config, catalog, materials, reg_params)` であり、`elements_geom` を受けるシグネチャではない。`frame_optimize/matlab/frameObjectiveFunction.m:1-19`

このため、**少なくとも `runFrameOptimization.m` 系は API ドリフトを起こしている**。

同じ問題は関連デモ・テストにも残っている。

- `demo_full_pipeline.m` は `runFrameOptimization(...)` を使う。`frame_optimize/examples/demo_full_pipeline.m:244`
- `test_week2_functions.m` は `frameObjectiveFunction` に旧引数列を渡している。`frame_optimize/tests/test_week2_functions.m:117-157`
- `test_frame_obj.m` は `results.f_stiff` を参照しているが、現行 `frameObjectiveFunction` が返しているのは `P_stiff` である。`frame_optimize/tests/test_frame_obj.m:62-71`, `frame_optimize/matlab/frameObjectiveFunction.m:275-291`

要するに、**現行主入口の外側に、壊れた旧経路が複数残っている**。

## 4. 規則チェック・適合性保証の主張と、実運用の実態がズレている

`frame_optimize/README.md` では、フレーム最適化を「FSAE 2026年規則への自動適合性チェック機能を備えた」「高速 MEX FEM ソルバー」ベースのシステムとして説明している。`frame_optimize/README.md:3-15`  
また README 内の進捗表では、規則チェッカーや除外ゾーンなどが「第2-4週 進行中」とも書かれており、文書内でも状態が揺れている。`frame_optimize/README.md:18-37`

一方、現行主入口 `run_frame_optimization.m` と目的関数 `frameObjectiveFunction.m` の実際の中核は **TCP/Java FEM サーバ**であり、MEX 中心の説明とは一致しない。`frame_optimize/examples/run_frame_optimization.m:274-283`, `frame_optimize/matlab/frameObjectiveFunction.m:171-179`

また、V2 以前の規則チェックの実装も一枚岩ではなかった。

- 主目的関数で毎回入っていたのは `checkMaterialRestriction(...)` と `checkExclusionZones(...)` が中心だった
- 包括的な `checkFrameCompliance.m` は別に存在したが、主経路からの接続が弱かった
- V2 では `frameObjectiveFunction.m` から `checkFrameCompliance(...)` を呼び、さらに `check_frame_geometry_constraints_v2.m` を FEM 前に挟む

つまり、V2 で規則チェックは主経路に近づいたが、driver/cockpit template の厳密判定や応力評価までは未完であり、まだ「完全な車検保証」ではない。

## 5. artifact には詳細を保存するが、下流の主経路で効くのは主に `K_chassis`

`save_frame_optimization_artifact.m` は、フレーム最適化結果を `params.frame` として詳細保存しつつ、`torsional_stiffness_Nm_per_deg` が有効なら `params.suspension.K_chassis` も上書きする。`matlab/save_frame_optimization_artifact.m:34-41`

保存内容自体はかなり豊富で、以下が入る。

- 選択パイプ群 `selected_pipes` `matlab/save_frame_optimization_artifact.m:112-131`
- 設計変数 `design_variables` `matlab/save_frame_optimization_artifact.m:85-91`
- ノード・connectivity `geometry` `matlab/save_frame_optimization_artifact.m:93-109`
- 性能指標と一部ターゲット `performance`, `targets` `matlab/save_frame_optimization_artifact.m:69-83,134-149`

ただし、下流の代表的な車両設計適用経路 `apply_full_design.m` は、フレーム詳細を読むのではなく、設計変数 `x(31)` を `Chassis Torsional Stiffness` として扱っている。`matlab/apply_full_design.m:41-42,65-74`

そのため現状は、**artifact に `params.frame` を厚く保存しても、標準的な下流利用が広く接続されているわけではない**。少なくとも代表的な導線では、効果が直接見えやすいのは `K_chassis` の更新側である。

## 6. 失敗時の扱いが「警告で続行」寄りで、前提不足を埋めにくい

フレーム最適化はサスペンション hardpoint 依存だが、その前提が満たされない場合でも hard error にはならない。

- `resolve_suspension_geometry_source` は hardpoint 付き artifact が見つからないと `warning(...)` で空文字を返す。`frame_optimize/matlab/resolve_suspension_geometry_source.m:19-30`
- `run_frame_optimization.m` はその戻り値を `config.suspension_file` に入れて続行する。`frame_optimize/examples/run_frame_optimization.m:43,155-156`
- `frameObjectiveFunction.m` 側では、rear 側が無いと固定ノードを既定値 `[9,10]`, `[13,14]` に落とす。`frame_optimize/matlab/frameObjectiveFunction.m:144-155`
- 質量が geometry から読めなければ `230 kg` を既定値にする。`frame_optimize/matlab/frameObjectiveFunction.m:161-165`

この方針だと、**前提不足のまま最適化が走ってしまい、「実車反映つもりの計算」が実は代替仮定で回っていた**という事故が起こりやすい。

## 7. 保存時バリデーションが弱く、静かに丸め込む箇所がある

`save_frame_optimization_artifact.m` は出力整理としては有用だが、入力整合性の検証は強くない。

- パイプ選択は `bestX` を `round` して catalog 範囲に clamp する。`matlab/save_frame_optimization_artifact.m:119-131`
- `extractVectorSlice` は不足長でも空/短いベクトルをそのまま返す。`matlab/save_frame_optimization_artifact.m:164-170`
- `getStructFieldOrDefault` は未設定を既定値へ落とすが、欠落フィールドの記録はしない。`matlab/save_frame_optimization_artifact.m:173-178`

このため、**壊れた最適化結果を落とすよりは保存を優先する設計**になっており、後から artifact を見ても、どこが既定値混入だったか追いにくい。

## 8. 目的関数のチューニング値がコード直書きで、再評価の再現性が弱い

目的関数には、コード内に直接埋め込まれた閾値が残っている。

- V1 互換経路では剛性ペナルティ既定値 `gamma_below = 8.0`, `gamma_hard = 50.0` が残る
- V2 経路では `run_canonical_frame_optimization.m` が `gamma_below`, `gamma_hard`, `gamma_mass`, `sparsity_penalty_scale`, `max_cg_z` を設定し、artifact の `objective_config` に保存する
- V2 の frame-only CG は車両 CG そのものではないため、硬い 0.380 m 制約ではなく、現状は 0.550 m の安全側上限と軽い `W_cg` 項で扱う

V2 では artifact の `objective_config` に penalty 係数、sparsity 閾値、荷重ケース変位閾値、optimizer mode / seed も保存するようにした。ただし、V1 artifact や過去結果ではこれらが欠落している可能性がある。

そのため、**V2 以降は再現性が改善したが、過去 artifact と旧経路は目的関数バランスを完全再現しにくい**。

## 9. 文書が強気すぎて、現実の運用状態を誤解しやすい

`frame_optimize/IMPLEMENTATION_COMPLETE.md` では「実装完了」「完全動作」と強く述べている。`frame_optimize/IMPLEMENTATION_COMPLETE.md:1-10`  
同文書では「FSAE規則準拠」「全テスト合格」「2-3ms/評価」といった成果が並ぶ。`frame_optimize/IMPLEMENTATION_COMPLETE.md:62-71,74-106`

しかし、同じリポジトリ内には次の実態がある。

- README は still `第2-4週 進行中` を含む。`frame_optimize/README.md:18-37`
- 主入口は対話依存で、自動化に不向き。`frame_optimize/examples/run_frame_optimization.m:115-129,337-350,462-467`
- 旧入口・旧テストは API ドリフトを抱えている。`frame_optimize/matlab/runFrameOptimization.m:39`, `frame_optimize/tests/test_week2_functions.m:117-157`

つまり、**サブディレクトリ内の文書同士でも「完成済み」認識が揃っていない**。

## 優先度つき整理

### 先に直すべきもの

1. **主入口の非対話化**  
   `input(...)` / `uiwait(...)` / base workspace `evalin(...)` 依存を外し、関数 API 化する。
2. **標準パイプラインへの接続整理**  
   `frame_optimized_vehicle_artifact.json` を `run_full_pipeline.m` がどう拾うか、または `optimized_*.json` 系へ揃えるかを決める。
3. **旧入口・旧テストの整理**  
   `runFrameOptimization.m` と関連 demo/test の API ドリフトを解消するか、非推奨化して入口を一つに絞る。

### 次に直すべきもの

1. **規則チェックの一本化**  
   `checkFrameCompliance.m` を使うなら主経路に接続し、署名不整合も直す。使わないなら削る。
2. **artifact の意味を明確化**  
   `params.frame` を下流で使うのか、保存専用メタデータなのかを決める。
3. **目的関数パラメータの設定化**  
   penalty 係数と閾値を config / artifact に明示保存する。

## 補足

本調査では、**「フレーム最適化そのものの数値計算が全く成立していない」とまでは判定していない**。  
問題の中心は、**運用経路・文書・API・自動化・下流接続の弱さ**であり、チーム開発の観点ではここが現状最大のリスクになっている。
