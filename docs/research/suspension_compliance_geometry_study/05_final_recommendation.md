# 05. Final Recommendation

## 1. 結論

この repository で最初にやるべき拡張は、**現行 double-wishbone 系に対する compliance-aware inverse-analysis オプション** です。  
その上で geometry family を増やすなら、優先順位は次です。

1. **Rear: explicit toe-link / A-arm+toe-link family**
2. **Front: upper-side 主体の virtual kingpin / double-joint upright**
3. **Rear: trailing-link + lateral links + toe link の drive/brake-path 分離型**
4. **最後に generic multi-link framework**

**pushrod / pullrod は family ではなく actuation / load-path option として扱うべき** です。`suspension_kinematics_design/FSAE_Suspension_Design_JP.md:10-37`

## 2. なぜ compliance-aware から始めるべきか

### 2.1 repo がすでに “有効アライメント” を受けられるから

runtime 側は per-corner の

- `camber_vs_travel`
- `toe_vs_travel`
- `camber_vs_steer`
- `toe_vs_steer`
- `roll_camber_gain`
- `roll_steer_gain`
- `jacking_force_per_fy`
- `toe_per_load`
- `camber_per_load`
- `caster_per_load`
- `track_per_load`
- `wheelbase_per_load`

を受けられます。`cpp/include/fsae/vehicle/vehicle_params.hpp:140-160,438-499`, `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:308-337,812-875`

つまり今足すべきものは、**新しい runtime モデル** ではなく、**設計結果を既存 runtime schema へ export する設計系** です。

### 2.2 ユーザーが心配しているのは、まさに “幾何よりコンプライアンスが支配的になるか” だから

現状 solver は幾何だけです。`suspension_kinematics_design/a_arm_kinematics_solver.m:78-175`  
しかし車両全体では frame twist も steering compliance も入っています。`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:783-804,979-986,1448-1474`

したがって設計系だけ geometry-only のままだと、**最適化で作った camber gain / toe behavior が、実走 equivalent の有効アライメントに直結しない** 可能性が高いです。

## 3. compliance-aware inverse-analysis の具体案

## 3.1 モデル分離

拡張は 1 本の巨大 solver ではなく、次の 4 レイヤーに分けるのが安全です。

1. **Geometry family layer**
   - どのリンク構成か
   - どの hardpoint が自由変数か
   - どの拘束式を使うか
2. **Kinematic sweep layer**
   - heave / roll / steer に対する wheel pose を解く
3. **Compliance layer**
   - wheel load → wheel pose 変化を解く
4. **Export layer**
   - existing `kc_corners / kc_front / kc_rear` へ surrogate を書く

## 3.2 compliance 計算の最低限の式

初期フェーズでは full nonlinear FEA ではなく、**condensed stiffness model** で十分です。

各リンク・接合部に対して:

- 軸方向剛性: `k_axial = E A / L`
- 曲げが重要なら beam end stiffness を 6x6 で使用
- rod end / bushing / rack support / upright mounting は節点ばねとして 6x6 近似

これを assembled system にして、

`K_global * u = F_loadcase`

を解き、upright / wheel center / contact patch の変位と姿勢から

- `Δtoe`
- `Δcamber`
- `Δcaster`
- `Δtrack`
- `Δwheelbase`

を取ります。

その後、既存 runtime schema に合わせて

- `toe_per_load[Fx,Fy,Fz,Mx,My,Mz]`
- `camber_per_load[...]`
- `caster_per_load[...]`
- `track_per_load[...]`
- `wheelbase_per_load[...]`

へ回帰 / 差分化して export します。`cpp/include/fsae/vehicle/vehicle_params.hpp:150-160`

## 3.3 “K&C より compliance が支配的” の判定方法

ここは定量化できます。設計候補ごとに、代表 load case で

- 幾何由来の有効変化 `Δtoe_geom`, `Δcamber_geom`
- コンプライアンス由来の変化 `Δtoe_comp`, `Δcamber_comp`

を別々に出し、比率

- `r_toe = |Δtoe_comp| / max(|Δtoe_geom|, ε)`
- `r_camber = |Δcamber_comp| / max(|Δcamber_geom|, ε)`

を見ます。

さらに tire sensitivity まで踏み込むなら、

- `ΔFy_geom ≈ (∂Fy/∂γ) Δcamber_geom + (∂Fy/∂α) Δtoe_geom`
- `ΔFy_comp ≈ (∂Fy/∂γ) Δcamber_comp + (∂Fy/∂α) Δtoe_comp`

として

- `r_force = |ΔFy_comp| / max(|ΔFy_geom|, ε)`

を見れば、**“接地に効く本質量” でどちらが支配的か** を比較できます。

### 推奨 load case

最低限これを各 corner / axle で打つべきです。

1. pure bump / rebound (`Fz`)
2. mid-corner lateral (`Fy`)
3. braking-in-turn (`Fx + Fy + Mz`)
4. traction-on-exit (rear `Fx + Fy`)
5. curb / impact-like vertical + aligning moment (`Fz + Mz`)

`correlation/real_vehicle_correlation_config.json` も compliance rig / MBS export を想定しています。`correlation/real_vehicle_correlation_config.json:250-263`

## 4. geometry family の優先順位

| family | 何を解決しやすいか | repo との相性 | 難易度 | 優先 |
| --- | --- | --- | --- | --- |
| Wishbone + compliance | 現行課題の直接解決。アーム剛性・有効アライメント評価 | 最高 | 低〜中 | 最優先 |
| Wishbone + toe link | bump steer / toe control を切り分ける | 高い | 中 | 最初の新 family |
| Wishbone + virtual kingpin / double-joint upright | 短アームでも KPI / scrub / trail / packaging 自由度を増やす | 中〜高 | 中〜高 | 次の front family |
| Rear trailing-link hybrid | 駆動・制動反力経路、anti、rear toe compliance を整理しやすい | 中 | 中〜高 | その次 |
| Generic multilink | 幾何自由度最大 | 低い（今の stack を大きく壊す） | 高 | 最後 |

## 5. double-joint は “upper だけ” から始めるべきか

**はい。最初は upper-side 主体の virtual kingpin 拡張から始めるのが良い** です。

理由:

- 現行 pipeline の `upper/lower upright joint -> kingpin axis` という考え方を保ったまま自由度を増やせる
- front package で効く KPI / scrub / trail / kickback 対策に直結しやすい
- 全ジョイント多点化より optimizer の次元増加が緩い

逆に **upper/lower とも全面的に double-joint 化** すると、実質的には multi-link general case に近づき、拘束管理も最適化も一気に難しくなります。

## 6. pushrod / pullrod を別 family にしない理由

pushrod / pullrod は

- アームが持つ荷重の種類
- 曲げ主体か軸力主体か
- install ratio / damper packaging

を変えますが、**wheel center の基本拘束 topology 自体は wishbone のまま** です。  
したがって、geometry family としてではなく

- `actuation_type = direct / pushrod / pullrod`
- `load_path_mode = arm_loaded / pushrod_loaded`

のような別軸パラメータで持つ方が自然です。

これはユーザーの懸念である

- アーム長が稼げない
- アーム延長すると剛性が落ちる
- ダンパ荷重までアームが持ってしまう

のうち、特に後ろ 2 つに直接効きます。

## 7. 実装ロードマップ

## Phase 1: 既存 wishbone の精密化

やること:

1. `a_arm_kinematics_solver.m` を、少なくとも heave / roll / steer sweep を返す exact-ish solver に更新
2. compliance postprocess を追加
3. `kc_corners / kc_front / kc_rear` exporter を実装
4. `build_vehicle_artifact` 系で vehicle artifact へ保存

主な対象:

- `suspension_kinematics_design/a_arm_kinematics_solver.m`
- `suspension_kinematics_design/main_a_arm_optimizer.m`
- `suspension_kinematics_design/main_rear_a_arm_optimizer.m`
- `matlab/build_vehicle_artifact.m`
- `matlab/save_vehicle_artifact.m`
- `matlab/pou_hardpoint_design/export/update_vehicle_params_json.m`

## Phase 2: family plugin 化

新設推奨:

- `suspension_kinematics_design/+families/wishbone_family.m`
- `suspension_kinematics_design/+families/rear_toe_link_family.m`
- `suspension_kinematics_design/+families/virtual_kingpin_family.m`
- `suspension_kinematics_design/+families/rear_trailing_hybrid_family.m`
- `suspension_kinematics_design/+compliance/solve_corner_compliance.m`
- `suspension_kinematics_design/+export/export_kc_surrogates.m`

## Phase 3: front/rear 別の高自由度 family 追加

- front: double-joint upright / virtual kingpin
- rear: trailing-link hybrid
- front/rear: 必要に応じて toe-link 派生の拡張

## Phase 4: generic multilink

これは最後です。  
前段で

- artifact schema
- surrogate export
- validation load cases
- correlation workflow

を確立してからでないと、自由度が増え過ぎて比較不能になります。

## 8. 先にやってはいけないこと

1. **generic multi-link optimizer を最初に作る**
2. **pushrod / pullrod を geometry family として混ぜる**
3. **compliance を runtime 側だけに押し込んで設計系を geometry-only のままにする**
4. **目標値の source of truth を整理しないまま optimizer を複雑化する**

特に 4 は重要で、rear RC target の drift だけでもすでに兆候があります。`suspension_kinematics_design/main_rear_a_arm_optimizer.m:41-59`, `suspension_kinematics_design/tests/test_rear_a_arm_optimizer.m:54-61`

## 9. 最終判断

この repo では、

- **幾何 family の拡張**
- **荷重経路 / actuation の拡張**
- **compliance の定量化**

を分けて扱うのが正解です。

その前提で、最初の 1 本目は

**「wishbone 系を維持したまま、load-case ベースの compliance solve を導入し、既存 KC schema へ export する」**

です。  
そこまで出来れば、double-joint upright も rear trailing hybrid も、vehicle model 側をほぼ変えずに比較できるようになります。
