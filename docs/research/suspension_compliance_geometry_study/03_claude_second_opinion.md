# 03. Claude Second Opinion

## 実行条件

ユーザー要求は **Claude Opus 4.6** でしたが、この task runtime では Opus 4.6 / 4.5 が起動できませんでした。  
そのため、Claude 系の第二意見は **Claude Sonnet 4.6** に代替しています。

## 1. Claude 系統の主張

Sonnet の結論は、かなり実装寄りで保守的です。

1. **現行 stack は strict double-wishbone の 2D cross-section 近似**
2. **最初にやるべきは rear toe-link の追加**
3. **double-joint upright は modeling cost の割に FSAE での便益が小さいので後回し**
4. **rod-end 前提なら arm compliance 自体は小さく、支配的なのは frame side (`K_chassis`)**

## 2. 現行 stack 認識

Sonnet は、現行系を「厳密な double-wishbone / rigid upright / static-only evaluation」と捉えています。

- `a_arm_kinematics_solver.m` は upper/lower arm midpoint を使う 2D FVSA ベースで、tie-rod を直接解いていない。`suspension_kinematics_design/a_arm_kinematics_solver.m:28-30,78-175`
- front / rear optimizer とも chassis pivot を前後対称に置く 10 変数問題。`suspension_kinematics_design/main_a_arm_optimizer.m:54-105,157-205`, `suspension_kinematics_design/main_rear_a_arm_optimizer.m:67-109,212-247`
- rear export には toe-link がなく、front tie-rod との非対称がある。`suspension_kinematics_design/main_rear_a_arm_optimizer.m:142-201`, `suspension_kinematics_design/suspension_base_geometry.json:1-138`

この認識は手元監査とも一致します。

## 3. compliance-aware inverse-analysis への提案

Sonnet は compliance 導入を 3 段に分けていました。

### A. post-process correction

hardpoint 最適化後に peak load を与え、toe / camber の compliance 分を見積もって geometric target を補正する。

### B. cost penalty 化

`cost_function()` に `δtoe_compliance`, `δcamber_compliance` を直接罰則として入れる。

### C. `ComplianceSurrogate` 正式 export

最終的にはこれを推奨しており、`toe_per_load`, `camber_per_load` 等を既存 C++ schema へ流すのが正道だとしています。  
これは runtime 既存受け皿に乗るため、アーキテクチャ整合性が高いです。`cpp/include/fsae/vehicle/vehicle_params.hpp:150-160,438-499`, `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:312-337,819-833`

## 4. Sonnet が重視した計算根拠

Sonnet が特に強調したのは「まず線形 compliance matrix から始める」ことでした。

- 軸方向: `δ = L / (E A) * F`
- 曲げ方向: `δ = F L^3 / (3 E I)`
- そこから upright / ball joint 位置摂動を toe / camber へ変換

この点は、初期フェーズで FEM まで行かずに設計 loop に入れたいという意味で合理的です。

## 5. geometry family に対する見解

Sonnet の優先順位は以下でした。

1. **A-arm + toe link (rear)**
2. pullrod variant
3. medium-term で rear multilink
4. double-joint upright は defer
5. trailing-arm hybrid は非推奨

この序列の背景は、

- current rear stack に toe control の別自由度がない
- POU/pullrod は実装コストが低い
- double-joint upright は rigid-kingpin 仮定を壊す

という repo 依存の事情です。

## 6. Sonnet 独自の強い主張

### 6.1 arm compliance は小さい可能性が高い

Sonnet は、FSAE の rod-end wishbone を前提にすると **arm compliance は全 alignment change の 5% 未満** で、むしろ `K_chassis` 由来の frame side compliance が支配的になりやすいと推定しました。

これは重要な反論で、ユーザーの懸念をそのまま肯定するのではなく、

- 本当に arm compliance が支配的か
- まず frame / pickup compliance を見るべきではないか

という観点を入れています。`cpp/include/fsae/vehicle/vehicle_params.hpp:186-190`, `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:768-780,979-986,1448-1474`

### 6.2 double-joint upright に懐疑的

Sonnet は、double-joint upright は modeling cost が高く、FSAE での benefit は通常の hardpoint placement で代替可能と見ています。  
この点は本調査の最終結論とは少し温度差がありますが、「最初にやるべきではない」という意味では妥当です。

## 7. 採用した点 / 保留にした点

### 採用

- `ComplianceSurrogate` 正式 export を本線にすること
- rear toe-link を高優先で評価対象に入れること
- pullrod を family ではなく別軸 option とみなすこと
- `K_chassis` と suspension target の coupling を意識すること

### 保留

- 「double-joint upright の benefit は marginal」という強い否定
- 「arm compliance は通常 5% 未満」という magnitude の一般化

この 2 点は、実際のアーム断面、アップライト剛性、frame node stiffness が未入力なので、最終文書では**断定ではなく仮説**として扱います。
