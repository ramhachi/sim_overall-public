# 02. GPT-5.4 Report

## 1. GPT-5.4 の総括

GPT-5.4 系統の結論は、かなり本 repo の実装構造に沿っています。

1. **現行 MATLAB 設計系は rigid A-arm problem に留まっている**
2. **C++ runtime schema はそれよりかなり先に進んでいて、KC/compliance surrogate の受け皿がある**
3. **最短経路は、その gap を埋めて `kc_corners / kc_front / kc_rear` を正規 export すること**
4. **最初の新 family は rear toe-link / rear toe-control 系**
5. **front の double-joint upright / virtual kingpin は relevant だが medium-term**

## 2. 現行 stack の把握

GPT-5.4 は、現行 stack を以下のように捉えました。

- core solver は **single-corner double wishbone only**。`U_F/U_R/U_U`, `L_F/L_R/L_U`, wheel center, tire radius だけを前提にしている。`suspension_kinematics_design/a_arm_kinematics_solver.m:1-10`
- front-view 解析では chassis pickup の前後 2 点を midpoint へ潰すため、true 3D hardpoint solve ではなく FVSA 近似になる。`suspension_kinematics_design/a_arm_kinematics_solver.m:28-31,78-109`
- kingpin は upper/lower outer joint の直線で固定され、virtual kingpin, toe link, multilink closure は存在しない。`suspension_kinematics_design/a_arm_kinematics_solver.m:32-58`
- front optimizer は 10 変数、front RC/KPI/caster をスクリプト内で hard-code している。`suspension_kinematics_design/main_a_arm_optimizer.m:27-40,54-60`
- rear optimizer も同型だが、`w_rc = 0.0` のため stated target RC は目的関数では実質無効。`suspension_kinematics_design/main_rear_a_arm_optimizer.m:258-269`

この評価は、現行 stack が **幾何学探索としては使えるが、別 family や compliance-aware 解析の基盤としては狭い** ことを示しています。

## 3. GPT-5.4 が強く見た architectural gap

GPT-5.4 が最も重視したのは、**runtime schema が設計系より先に進んでいる** 点です。

- `VehicleParams` には hardpoints, kinematic surrogates, compliance surrogates, axle metrics が定義済み。`cpp/include/fsae/vehicle/vehicle_params.hpp:117-190,460-500`
- `Vehicle14DOF_New` は travel / steer / roll / load に応じて effective toe / camber を既に使っている。`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:303-337,812-833`
- correlation 側にも `toe_per_load`, `camber_per_load`, `K_chassis`, static alignment, camber gain の計測口がある。`correlation/real_vehicle_correlation_config.json:151-170,173-210,240-263`

したがって GPT-5.4 の立場は一貫していて、**新 family を増やす前に “設計結果 -> runtime KC schema” を閉じるべき** というものです。

## 4. compliance-aware inverse-analysis への提案

GPT-5.4 は、最も実装整合性が高い案として

`effective alignment = geometric term + load-dependent compliance term`

の surrogate export を推しました。

具体的には:

- `toe_per_load`
- `camber_per_load`
- 必要なら `caster_per_load`, `track_per_load`, `wheelbase_per_load`

を設計結果として出し、既存 C++ runtime の `kc_corners` へ流します。`cpp/include/fsae/vehicle/vehicle_params.hpp:150-160`, `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:325-327,829-831`

### compliance dominance の判定

GPT-5.4 は、コンプライアンス優勢かどうかを **load envelope 上で**

- `|Δα_comp| >= |Δα_geom|`

で判断すべきだと整理しました。  
ここで `α` は toe / camber / caster などの有効アライメントです。

判断材料として挙げたのは:

- lateral / brake load 下の toe/camber 変化が bump/roll の geometric 変化と同等以上
- step-steer / skidpad の相関が compliance surrogate を入れないと合わない
- SAT 依存の steering response が rigid tie-rod 幾何で説明できない
- frame twist が front/rear の “見るロール” を変えてしまう

というもので、**机上の幾何値ではなく vehicle-level correlation で確かめる** 立場です。

## 5. geometry family の優先順位

GPT-5.4 の優先順位は次でした。

1. **explicit toe-link / rear toe-control family**
2. **double-joint upright / virtual kingpin (front)**
3. pushrod / pullrod packaging variants
4. rear trailing-link hybrid
5. full multilink

### rear toe-link を先に置く理由

- current rear export に toe-link がない
- rear passive steer は vehicle behavior への効きが大きい
- existing runtime の toe-related surrogate を自然に活かせる

### double-joint upright を中期候補に置く理由

- scrub / KPI / trail / steering-axis packaging を分離できる
- “全く異なる geometry” としては魅力がある
- ただし current kingpin 仮定を壊すので、rear toe-link より実装コストは高い

## 6. GPT-5.4 のロードマップ

### near-term

1. geometry family flag と schema versioning
2. richer KC export
3. compliance-aware target cases
4. rear toe-link family
5. pushrod / pullrod option

### medium-term

1. double-joint upright / virtual kingpin
2. reduced structural compliance
3. multi-case vehicle-level sensitivity optimization

### long-term

1. generic family registry
2. multilink / trailing-link hybrid families
3. flexible-body / MBS-backed identification

## 7. GPT-5.4 から採用した点

- **“runtime schema is ahead of MATLAB design stack”** という framing
- compliance を既存 `kc_corners` に合わせて export する方針
- rear toe-link を first new family に置くこと
- double-joint upright は relevant だが medium-term に落とすこと
- correlation / measurement を経て compliance dominance を判断すること

## 8. Sonnet との差

主な差は 2 つです。

1. **double-joint upright の評価**  
   Sonnet はやや懐疑的、GPT-5.4 は medium-term で十分 relevant とみています。

2. **arm compliance の効き量**  
   Sonnet は rod-end 前提なら小さいと推定、GPT-5.4 は magnitude を決め打ちせず、load-case と correlation で判定すべきとしています。

最終判断では、後者の慎重姿勢を採用します。
