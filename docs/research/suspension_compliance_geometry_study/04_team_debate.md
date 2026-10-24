# 04. Team Debate

このファイルでは、

- **GPT-5.4**
- **Claude Sonnet 4.6**  
  （本来は Opus 4.6 指定だったが runtime 非対応のため代替）

の議論を比較し、最終採用判断を整理する。

## 1. 強く一致した点

### 1.1 現行 stack は A-arm 専用で、solver は簡略的

両者とも、

- current optimizer が strict wishbone / A-arm 前提
- `a_arm_kinematics_solver.m` は FVSA 的な簡略 solver
- tie-rod / toe-link / multilink / virtual kingpin は本体 solver には入っていない

という認識で一致しました。`suspension_kinematics_design/a_arm_kinematics_solver.m:28-31,78-175`, `suspension_kinematics_design/main_a_arm_optimizer.m:54-105`, `suspension_kinematics_design/main_rear_a_arm_optimizer.m:67-109`

### 1.2 runtime は設計系より先に進んでいる

両者とも、今回の本質を

**「C++ runtime の KC/compliance schema はすでにあるのに、MATLAB inverse-design がそこを埋めていない」**

と捉えました。`cpp/include/fsae/vehicle/vehicle_params.hpp:117-190,438-499`, `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp:303-337,812-875`

### 1.3 generic multilink を最初にやるべきではない

これは完全に一致しました。  
理由も共通で、

- 変数が爆発する
- source-of-truth が固まっていない
- validation が弱い

です。

### 1.4 pushrod / pullrod は別軸 option

両者とも、pushrod / pullrod は **新キネマティクス family ではなく load-path / packaging option** とみなしました。  
この点は本 repo の POU 分離思想とも整合します。`suspension_kinematics_design/FSAE_Suspension_Design_JP.md:10-37`

## 2. 見解が割れた点

### 2.1 arm compliance は本当に支配的か

#### Sonnet

- rod-end 前提の FSAE なら arm compliance は小さい可能性が高い
- 本当に効くのは `K_chassis` を介した frame side の柔さかもしれない

#### GPT-5.4

- magnitude を決め打ちせず、load-case と correlation で判定すべき
- “支配的かどうか” は `|Δα_comp|` と `|Δα_geom|` の比較で決めるべき

#### 採用判断

**GPT-5.4 の慎重側を採用** します。  
Sonnet の推定は有益ですが、断面・材質・upright 剛性・frame node stiffness が未入力なので、今の段階で “小さいはず” と固定するのは早いです。

ただし Sonnet の警告も残します。  
つまり final recommendation では、

- 最初の compliance solve は軽量 surrogate でよい
- しかし支配性の有無は **計算 + correlation** で確認する

とします。

### 2.2 double-joint upright の優先度

#### Sonnet

- rigid kingpin 仮定を大きく壊す
- modeling cost が高い
- benefit は marginal かもしれない

#### GPT-5.4

- first new family ではないが medium-term では十分 relevant
- KPI / scrub / trail / steering-axis packaging を切り分けられる

#### 採用判断

**“最初ではないが、明確な第2候補”** として採用します。  
つまり、

1. first new family = rear toe-link / A-arm+toe-link
2. first front-side new family = virtual kingpin / double-joint upright

とします。

## 3. 最終採用判断

### 3.1 最初に作るもの

**current wishbone family の compliance-aware export**

内容:

- exact-ish kinematic sweep を増やす
- load-case based surrogate を作る
- `kc_corners`, `kc_front`, `kc_rear` へ export する

### 3.2 最初の新 family

**rear toe-link / A-arm+toe-link**

理由:

- repo と最も相性がよい
- toe control を独立自由度にできる
- vehicle behavior に効きやすい

### 3.3 次に入れる front 側 family

**virtual kingpin / double-joint upright**

ただし段階は

- upper-side 主体の拡張から
- all-joint multi-body upright は後

が良いです。

### 3.4 その後

- pullrod / pushrod option
- rear trailing-link hybrid
- generic multilink

## 4. この議論から出る実装原則

1. **family, actuation, compliance を分ける**
2. **新スキーマを作る前に既存 runtime schema を使う**
3. **支配性は load-case 比較で判定する**
4. **front / rear を同じ順序で一般化しない**

この 4 点が、今回の team synthesis のコアです。
