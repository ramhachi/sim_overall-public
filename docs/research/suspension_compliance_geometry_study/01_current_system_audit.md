# 01. Current System Audit

## 1. 現行スタックの中心思想

現行 `suspension_kinematics_design/` は、名前どおり **A-arm / double-wishbone を前提** にした設計パイプラインです。

- フロントは `main_a_arm_optimizer.m` が `track / static camber / target roll center / KPI / caster / camber gain` を見ながら 10 変数で `fmincon + MultiStart` を回し、`suspension_base_geometry.json` を生成します。`suspension_kinematics_design/main_a_arm_optimizer.m:24-151`
- リアは `main_rear_a_arm_optimizer.m` が同型の 10 変数で RL/RR を追記します。`suspension_kinematics_design/main_rear_a_arm_optimizer.m:29-201`
- ステアリングは別段で `main_steering_optimizer.m` が tie-rod hardpoints を最適化します。つまり、**幾何学の主骨格 → tie-rod → POU** の順で積み上げる構造です。`matlab/steering_hardpoint_design/main_steering_optimizer.m:19-82`
- 公式説明文書もフロント A-arm / リア A-arm / POU の分離を前提にしています。`学生フォーミュラ車両運動シミュレーション総合ガイド.md:137-166`, `suspension_kinematics_design/Integration_Architecture_JP.md:9-57`

## 2. 現行 solver が実際に解いているもの

`a_arm_kinematics_solver.m` は、**完全な 3D compliant K&C solver ではありません**。

- ロールセンターは上下アームのシャーシ側前後ピボットの**中点**を使った front-view swing arm 近似です。`suspension_kinematics_design/a_arm_kinematics_solver.m:28-30,78-109`
- KPI / caster / scrub radius は upper/lower upright ball joint から作る kingpin 軸で計算しています。`suspension_kinematics_design/a_arm_kinematics_solver.m:32-76`
- 静的 camber は `Spindle_Dir` が与えられない限り厳密には決められず、コード中でも upright の第3自由度は tie-rod なしでは拘束し切れないと認めています。`suspension_kinematics_design/a_arm_kinematics_solver.m:111-137`
- camber gain も exact 3D 解ではなく、instant center からの swing-arm 長を用いた近似差分です。コード内コメントでも exact solve の不足を明示しています。`suspension_kinematics_design/a_arm_kinematics_solver.m:138-175`
- テストも「平行アームなら RC=0」「収束アームなら IC が有限」程度の解析単体テストが中心です。`suspension_kinematics_design/tests/test_a_arm_kinematics.m:6-43`

したがって現行 solver は、**A-arm 基本形の初期レイアウト探索** としては成立していますが、ユーザーが懸念している

- アーム延長時の剛性低下
- コンプライアンス優勢化
- 別形式による制約回避

を直接は扱っていません。

## 3. downstream での現実のデータフロー

### 3.1 `suspension_base_geometry.json` は重要だが、情報量はまだ薄い

- フロント optimizer は FL hardpoints を書き、リア optimizer は RL/RR を追記します。`suspension_kinematics_design/main_a_arm_optimizer.m:136-151`, `suspension_kinematics_design/main_rear_a_arm_optimizer.m:142-201`
- フロント geometry には tie-rod も保存されます。実際の JSON もその形です。`suspension_kinematics_design/suspension_base_geometry.json:1-54`
- `define_suspension_geometry.m` はこの JSON を読んで POU 側へ流し込みますが、実装は full double-wishbone hardpoints を前提にしています。`matlab/pou_hardpoint_design/geometry/define_suspension_geometry.m:1-31,100-155`

### 3.2 steering optimizer も簡略化されている

`main_steering_optimizer.m` は bump steer を「25 mm bump 時に ideal に上に動いた upright-side tie rod 点」との長さ差で評価しています。これは有効な first-pass ですが、**upright 6D motion / compliance / rack support stiffness / steering arm flexibility** は入っていません。`matlab/steering_hardpoint_design/main_steering_optimizer.m:85-116`

### 3.3 POU は “別キネマティクス形式” ではなく actuation layer

POU documentation は、double-wishbone がタイヤ軌跡を決め、pushrod / bellcrank が荷重支持と motion ratio を決めると明確に分離しています。`suspension_kinematics_design/FSAE_Suspension_Design_JP.md:10-37`

これは重要で、**pushrod / pullrod を geometry family と同列に扱うと設計責務が混ざります。**

## 4. いっぽう runtime 側はすでに richer な K&C / compliance を受けられる

ここが今回の最大の発見です。設計系は薄いのに、車両モデル側はすでに受け皿を持っています。

- `VehicleParams::SuspensionParams` には per-corner hardpoints, kinematic surrogates, compliance surrogates, axle-level roll-center / anti-geometry が定義済みです。`cpp/include/fsae/vehicle/vehicle_params.hpp:117-190`
- JSON parser も `kc.front / kc.rear / kc.corners` からそれらを読み込めます。`cpp/include/fsae/vehicle/vehicle_params.hpp:438-499`
- `Vehicle14DOF_New` は `camber_vs_travel`, `camber_vs_steer`, `roll_camber_gain`, `toe_vs_travel`, `roll_steer_gain`, `toe_per_load`, `camber_per_load`, `jacking_force_per_fy`, `roll_center_height_per_travel`, `roll_center_height_per_roll`, `anti_*` を実際に使います。`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:308-337,812-875`
- さらにフレームねじり状態 `phi_twist / dphi_twist` と `K_chassis / C_chassis / I_twist` もモデル化済みです。`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:77-85,768-780,979-986,1448-1474`; `cpp/include/fsae/vehicle/vehicle_params.hpp:186-190`

つまり、**「コンプライアンスを考慮した inverse-analysis を追加する」こと自体は runtime 側の schema に自然に接続できる** 状態です。

## 5. correlation 側も compliance 計測を想定している

`correlation/real_vehicle_correlation_config.json` には、すでに以下の受け口があります。

- `caster_deg`, `kpi_deg`, `scrub_radius_m`, `mechanical_trail_m` → `suspension.kc_corners[*]` `correlation/real_vehicle_correlation_config.json:195-221`
- `roll_center_heights_m`, `anti_geometry_pct` → `kc_front / kc_rear` `correlation/real_vehicle_correlation_config.json:223-249`
- `toe_per_load`, `camber_per_load` → `kc_corners[*].compliance.*` `correlation/real_vehicle_correlation_config.json:250-263`
- `hardpoints_file` → `kc_corners` / `pou_hardpoints` `correlation/real_vehicle_correlation_config.json:264-270`

method 欄も **“Compliance rig or MBS export”** を明記しています。つまり repository 全体の思想として、**最終的な truth source は hardpoint solve だけでなく K&C rig / MBS / 実験相関** です。

## 6. 現行スタックの主要ギャップ

### 6.1 family abstraction がない

front / rear optimizer はどちらも

- 2 本の A-arm
- upper / lower outer ball joint
- kingpin はそれらを結ぶ直線
- chassis mounts は前後対称

を暗黙前提に持っています。`suspension_kinematics_design/main_a_arm_optimizer.m:54-105,157-205`, `suspension_kinematics_design/main_rear_a_arm_optimizer.m:67-109,212-247`

そのため、

- double-joint upright
- A-arm + toe link
- trailing-link + lateral links
- general multi-link

を「同じ solver API」に入れる構造がありません。

### 6.2 compliance solver がない

現行設計系には

- link stiffness
- joint/bushing stiffness
- rack/support stiffness
- upright flexibility
- load-case-based wheel pose solve

がありません。したがって `kc_corners[*].compliance.*` を設計から埋める経路がありません。

### 6.3 docs / tests / targets に drift がある

注意点として、周辺文書にはいくつかズレがあります。

- `Integration_Architecture_JP.md` は A-arm optimizer を「新規開発が必要」と書いていますが、実体はすでに存在します。`suspension_kinematics_design/Integration_Architecture_JP.md:34-40,61-70`, `suspension_kinematics_design/main_a_arm_optimizer.m:1-151`
- `main_rear_a_arm_optimizer.m` の target RC は `-0.025 m` ですが、テストは `-0.050 m ± 15 mm` を期待しています。`suspension_kinematics_design/main_rear_a_arm_optimizer.m:41-59`, `suspension_kinematics_design/tests/test_rear_a_arm_optimizer.m:54-61`

この drift は、geometry-family 拡張前に **目標値定義・検証基準の source of truth を整理すべき** ことを示しています。

## 7. 監査から出る実装含意

1. **今の設計系を全部捨てる必要はない。**  
   むしろ current wishbone optimizer の上に exact kinematic sweep と compliance postprocess を足すのが最短です。

2. **最初に generic multi-link optimizer を作るのは危険。**  
   先に exporter / schema / validation を既存 KC schema に沿って固めるべきです。

3. **pushrod / pullrod は geometry family ではなく load-path option。**  
   キネマティクス solver とは分け、compliance solver に効かせるべきです。

4. **double-joint upright は front に非常に相性が良い。**  
   kingpin / KPI / scrub / trail の自由度を増やしつつ、wishbone ベースの pipeline を大きく壊さずに拡張できます。

5. **rear は “リンク形式の差” が vehicle behavior に効きやすい。**  
   駆動・制動反力と toe compliance を分けられる trailing-link hybrid 系が、front より導入価値が高い可能性があります。
