# FSAE フレーム最適化プログラム (MATLAB版)

## 概要

本プロジェクトは、Formula SAE（FSAE）のスペースフレーム設計のためのMATLABベースの最適化フレームワークです。トポロジーおよび断面形状の迅速な検討を可能にし、FSAE 2026年規則への自動適合性チェック機能を備えています。

### 現行の正規フロー（SSOT）

* **正規入口**: `frame_optimize/examples/run_frame_optimization.m`
* **主目的関数**: `frame_optimize/matlab/frameObjectiveFunction.m`
* **成果物保存**: `matlab/save_frame_optimization_artifact.m` を介して vehicle artifact JSON へ統合
* **入力前提**: サスペンション hardpoints は既存の vehicle artifact（EasyStart [8] 系）から解決する
* `frame_optimize/matlab/runFrameOptimization.m` は旧デモ互換のラッパーであり、新規の主入口としては扱いません

### V2 フレーム設計 schema

V2 は既存 18 変数の V1 と互換を残した追加経路です。`run_canonical_frame_optimization(..., 'UseFrameDesignV2', true)` で有効化し、次を設計変数化します。

* FBH / Front Hoop / Main Hoop / Shoulder Bar / Main Hoop Brace / リア終端台形の主要寸法
* `front_hoop`, `main_hoop`, `side_impact`, `front_bulkhead`, `fbh_support`, `front_susp_mount`, `mh_bracing`, `mh_brace_support`, `shoulder_bar`, `component_protection`, `diff_mount`, `rear_box`, `secondary_brace` の規定語彙別 pipe index
* Main Hoop Brace 終端は必須の台形構造とし、その後方のみデフマウント・リアサス接続用の自由領域として扱う

V2 は `params.frame.diff_mount_points` または `params.powertrain.diff_mount_points` が artifact にない場合、仮値で走らせず fail-fast します。評価ケースは torsion に加え、lateral / braking / rollover / diff reaction を生成し、V2 では全ケースの変位・剛性メタデータを `params.frame.load_cases` に保存します。

Optimizer は既定互換の `mixed_ga` のままですが、`OptimizerMode` で `mixed_ga`, `matlab_ga`, `lhs_local`, `nsga2_like`, `local_refine` を選択できます。比較用には `run_frame_optimizer_comparison.m` が同一評価予算・seed 群で Markdown レポートを出力します。

#### V2 MATLAB GA 固定評価数ベンチマーク

15000 評価規模の計測と最終フレーム可視化は次の入口で実行できます。

```bash
bash scripts/run_matlab_batch.sh "addpath(fullfile(pwd,'frame_optimize','examples')); r=run_frame_v2_targeted_ga_budget('EvaluationBudget',15000,'PopulationSize',100,'TargetStiffnessNmPerDeg',7000,'MassTargetKg',25,'RandomSeed',7000,'OutputDir',fullfile(pwd,'output','runtime','frame_v2_ga_15000'));"
```

この入口は内側の探索を `torsion_only` で回し、最終個体だけを torsion / lateral / braking / rollover / diff reaction の全ケースで再評価します。`SolverBackend='mex'` が既定で、MEX が見つかる場合は Docker/TCP FEM server を起動しません。

2026-05-10 の初回 15000 評価実測では MATLAB `ga` が 15020 評価、最終確認込み `actual_funccount=15021`、322.8 s、0.0215 s/eval でした。ただし目的関数と topology が重量を軽く見すぎ、最終個体は 66.96 kg でした。

同日修正後の安定化スモークでは、`output/runtime/frame_v2_ga_300_mex_final` が 309 評価、61.2 s、0.1980 s/eval、finite score rate 100%、true valid individual rate 62.5%、65 nodes / 97 elements、45.06 kg、4909 Nm/deg、規定チェック pass です。MEX 単体は同フレームで median 0.001432 s/eval のため、残る速度課題は FEM kernel ではなく MATLAB の目的関数周辺・規定チェック・GA overhead です。

重量 25 kg / 7000 Nm/deg 目標にはまだ未達です。現状の V2 は「Docker 依存と Inf population の破綻は解消、ただし current mandatory topology では質量下限が高い」という位置付けです。

#### V2 28 kg トポロジー下限確認

固定 97 要素 topology だけを見ると、最軽量規定管の割当でも 31.03 kg / 3479 Nm/deg で 28 kg を超えます。
一方、規定管に近い sparse topology へ差し替える実験入口を追加しています。

```matlab
addpath(fullfile(pwd, 'frame_optimize', 'examples'));
experiment_frame_v2_mass_cap_stiffness( ...
    'ArtifactPath', fullfile(pwd, 'output', 'runtime', ...
        'frame_v2_ga_300_mex_final', 'frame_v2_ga_15000_vehicle_artifact.json'), ...
    'MassCapKg', 28, ...
    'TopologyMode', 'regulation_minimal', ...
    'ConnectDiffMounts', true);
```

2026-05-11 の実測では、diff mount ありの sparse topology が 61 nodes / 82 elements、最軽量規定管で 27.23 kg / 3357 Nm/deg でした。diff mount を外し suspension bracket 接続だけにすると 57 nodes / 74 elements、24.28 kg / 3357 Nm/deg です。ただし現 V2 の Upper Side Impact node model は F.6.4 を完全表現していないため、この値は最終車検適合ではなく topology 下限確認として扱います。

**主な特徴:**

* **高速MEX FEMソルバー**: 1回あたりの評価時間は5ms未満（100要素、C++ + Eigen3）
* **FSAE 2026 適合性確認**: Size A/B/C/D の自動判定 (F.3.4.1)
* **市販パイプカタログ**: 実世界の入手性を考慮した離散最適化
* **多目的最適化**: 質量 vs 職性剛性のトレードオフ検討
* **接合部補正**: CIDECT公式 + コープカット（えぐり加工）の実験的係数
* **除外ゾーン（Exclusion Zones）**: AABB/シリンダーによるクリアランス確認（未対応typeはエラー）

---

## プロジェクト状況

### ✅ 第1週 完了 (2026-02-24)

* [x] JSONデータファイル（材料、パイプカタログ、規則）
* [x] MATLABデータローダー（材料、カタログ、断面特性）
* [x] MEX FEMソルバー（C++、3Dティモシェンコ梁、接合部補正）
* [x] MEXビルドスクリプト（macOS/Linux/WindowsでのEigen3自動検出）
* [x] サイズクラス判定 (FSAE F.3.4.1)
* [x] シンプルなフレームデモ（16ノード、ねじり剛性テスト）

### 🚧 第2-4週 進行中

* [ ] 規則チェッカー (第2週)
* [ ] 除外ゾーン機能 (第2週)
* [ ] 遺伝的アルゴリズム（GA）最適化 (第3週)
* [ ] グラウンドストラクチャ法 (第3週)
* [ ] 3Dビジュアライゼーション (第4週)
* [ ] PDFレポート生成 (第4週)

---

## インストール

### 必要要件

**MATLAB:**

* MATLAB R2018a 以降 (C++14 MEX サポートのため)
* Optimization Toolbox (`ga`, `particleswarm` 用)
* MATLAB Report Generator (任意、PDFレポート用)

**C++ ビルドツール:**

* Eigen3 ライブラリ (線形代数用)
* C++14 対応コンパイラ (GCC, Clang, MSVC)

### セットアップ手順

#### 1. Eigen3 のインストール

**macOS (Homebrew):**

```bash
brew install eigen

```

**Linux (Ubuntu/Debian):**

```bash
sudo apt-get install libeigen3-dev

```

**Windows:**

1. [https://eigen.tuxfamily.org/](https://eigen.tuxfamily.org/) から Eigen3 をダウンロード
2. `C:\eigen3` に展開
3. パスに `Eigen/Dense` ヘッダーが含まれていることを確認

#### 2. リポジトリのクローン

```bash
cd ~/projects/FomulaTMU/sim_overall
# frame_optimize/ ディレクトリが存在することを確認

```

#### 2.5. クロスプラットフォーム FEM バイナリのビルド

フレーム最適化の TCP/スタンドアロン実行は、`frame_optimize/bin/beam_fem_server` と
`frame_optimize/bin/beam_fem_standalone` を優先して使います。Windows では `.exe` を自動検出し、
ローカル実行できない場合のみ Docker にフォールバックします。

```bash
# macOS / Linux / WSL
./setup_env.sh build

# Windows
setup_env.bat
```

#### 3. MEX ソルバーのビルド

MATLABを開き、以下を実行：

```matlab
cd frame_optimize/matlab
build_mex()

```

出力例:

```
Building MEX FEM solver...
Found Eigen3 at: /opt/homebrew/include/eigen3
Compiling with command:
  mex -v -R2017b -I/opt/homebrew/include/eigen3 ...
✓ MEX compilation successful!

```

#### 4. デモの実行

```matlab
cd ../examples
run_frame_optimization

% 非対話（CI・自動化）
run_frame_optimization('Interactive', false, ...
    'VehicleParamsFile', '/path/to/vehicle_artifact.json', ...
    'MassTargetKg', 30, 'MaxGenerations', 5, 'UseParallel', false);

```

非対話モードでは入力プロンプトを出さず、viewer/`uiwait` も無効になります。
更新先の vehicle artifact は `result.output_path` と `frame_optimization_output_path` で確認できます。

---

## プロジェクト構造

```
frame_optimize/
  data/                         # JSON データファイル
    materials.json              # 材料特性 (AISI 4130, Al 6061 等)
    pipe_catalog.json           # 市販パイプ (17エントリ)
    regulation_params.json      # FSAE 2026 Size A/B/C/D 閾値

  mex/                          # C++ MEX ソース
    beam_fem_solver.cpp         # 3D ティモシェンコ梁 FEM (Eigen3)

  matlab/                       # MATLAB 関数
    loadMaterials.m             # JSONからの材料読み込み
    loadPipeCatalog.m           # カタログ読み込み + 断面特性計算
    frame_design_schema_v2.m     # V2設計変数schema
    decode_frame_design_vector.m # V1/V2 design vector decode
    generate_parametric_nodes_v2.m
    generate_adaptive_topology_v2.m
    check_frame_geometry_constraints_v2.m
    run_frame_optimizer_mode.m   # mixed_ga / matlab_ga / lhs_local / nsga2_like / local_refine
    computeTubeSectionProperties.m  # A, Iy, J の計算
    determineSizeClass.m        # FSAE Size A/B/C/D 判定
    build_mex.m                 # MEX コンパイルスクリプト

  examples/                     # デモスクリプト
    demo_simple_frame.m         # 16ノードフレームねじりテスト
    run_frame_optimization.m    # 正規のSSOTフロー

  tests/                        # ユニットテスト / 正規フローのスモークテスト

  output/                       # 結果出力ディレクトリ

```

---

## FSAE 規則への適合

### サイズクラス (F.3.4.1)

| クラス | I_min [mm⁴] | A_min [mm²] | D_min [mm] | t_min [mm] | 主な用途 |
| --- | --- | --- | --- | --- | --- |
| **A** | 11,320 | 173 | 25.0 | 2.0 | Main Hoop, Front Hoop, Shoulder Harness Mounting Bar |
| **B** | 8,509 | 114 | 25.0 | 1.2 | Front Bulkhead, Side Impact, Main Hoop Bracing |
| **C** | 6,695 | 91 | 25.0 | 1.2 | Front Bulkhead Support, MH Brace Support, Structural Tubing |
| **D** | 18,015 | 126 | 35.0 | 1.2 | Bent/Multi-tube Upper Side Impact |

※ サイズクラス認定には、**4つの基準（I, A, D, t）すべてを満たす**必要があります。

### 材料制限 (F.3.2.1)

* **鋼鉄限定構造**: メインフープ、メインフープブレイシング、ショルダーハーネスマウンティングバー
* **アルミニウム厚さ**: 最小 2.0mm (非溶接)、3.0mm (溶接)
* **注意**: `checkFrameCompliance.m` は Size D を A/B/C の単純な強弱序列ではなく、`I/A/D/t` の実寸閾値で判定します。

---

## パフォーマンス目標

| 指標 | 目標値 | 第1週ステータス |
| --- | --- | --- |
| MEX 計算時間 (100要素) | <5ms | ✅ 約3ms |
| 目的関数計算 | <10ms | ⏳ 第2週予定 |
| GA最適化 (50世代) | <30s | ⏳ 第3週予定 |
| シェルFEMとの精度誤差 | ±10% | ⏳ キャリブレーションが必要 |

---

## 次のステップ (第2週)

1. **規則チェッカー** (`checkFrameCompliance.m`):
* Size A/B/C/D の検証
* 材料制限の強制適用
* 必須構造の存在チェック
* ペナルティ関数の統合


2. **除外ゾーン** (`checkExclusionZones.m`):
* AABB (エンジン、バッテリー)
* シリンダー (排気系クリアランス)
* 未対応typeはfail-fast（凸包は未実装）
* 干渉量の計算


3. **目的関数** (`frameObjectiveFunction.m`):
* MEXソルバー + 規則 + 除外ゾーンの統合
* 閾値ベースの剛性報酬設定
* 多目的最適化: 質量 60%、剛性 40%



---

## FEM physics model (2026-10, Epic #1)

Kernel: `cpp/beam_fem_core.hpp`, shared by `beam_fem_standalone`, `beam_fem_server` and the MEX.
Closed-form checks: `beam_fem_standalone --selftest` (CTest `beam_fem_closed_form`) and
`tests/test_beam_fem_closed_form.m`.

* **Element row** `[ni nj D t E G rho shape jf kAy kAz off_i off_j]` (D, t, kA in mm/mm², offsets in m).
  11-column rows have no offsets; legacy 9-column rows run Euler–Bernoulli (Φ = 0) with a warning.
* **Section (#47)**: round J = πD⁴/32·(1−(Di/D)⁴), kA = A/2 (Cowper); square J = b³t (Bredt, b = D − t,
  sharp corners because the catalog lists no corner radius), kA = 2bt (web area).
* **Timoshenko (#46)**: Φ = 12EI/(G·kA·L²) form, same DOFs.
* **Joint model (#48)**: `compute_joint_corrections.m` gives each member a CIDECT-style factor
  `jf = clip(1 − 0.15(γ/10)(1−√β)/sinθ, 0.5, 1)` (mean of both ends, chord = largest OD at the node,
  members within 30° of the chord are continuous) and rigid end offsets r_chord/sinθ (flexible length
  L − off_i − off_j). Bonded CFRP members get no weld factor. `config.joint_factor` (scalar) overrides jf.
  Note: for equal-OD joints (β = 1) the ported formula gives jf = 1; calibrate with the override
  (docs.md suggests 0.55–0.70) until stage 2 exists.
  *Stage 2 (follow-up, not implemented)*: per-joint-topology 6×6 condensed stiffness from offline shell
  sub-models, inserted as joint springs at beam nodes (≈3× DOF).
* **Load cases (#49)**: rollover = rule main-hoop case (6 / 5 / −9 kN, verify against the current rules)
  at the main-hoop top node (`meta.main_hoop_top_idx`). `config.load_cases_file` (schema
  `frame_load_cases/v1`, see `import_frame_load_cases.m`) replaces the fixed-g lateral / braking /
  diff proxies with per-mount forces/moments, e.g. a 14DOF envelope; A-arm ball-joint forces are split
  along the two legs onto the chassis pickups; the case keeps the file's `source` tag. Supports: 3-2-1 on
  the rear mounts (reactions carry the inertial resultant). `data/frame_load_cases_synthetic_example.json`
  is **synthetic** (schema demo only); no 14DOF mount-load export exists yet.
* **Stress recovery (#50)**: solvers return `element_forces` (Mx12 local end forces at the flexible ends),
  `element_stress_vm` / `stress_max` (extreme-fibre |N|/A + bending with torsion, von Mises, MPa) and
  `element_buckling_ratio` / `buckling_ratio` (compression / Euler P_cr, K = 1, L = flexible length).
  The objective adds `gamma_yield·Σmax(0, σ_vm/Sy_welded − 1)² + gamma_buckling·Σmax(0, P/P_cr − 1)²`
  over all non-torsion cases (defaults 50/50). SES equivalence uses `rulebook_baseline` in
  `data/materials.json` (F.3.4.2 values 200 GPa / 305 / 365 MPa, verify against the current rules).
  Not yet: transverse shear stress, local D/t buckling, fatigue (rainflow/Miner is P1 post-processing).
* **Constraints / attachments (#52)**: BCs are eliminated (no 1e15 penalty); `rigid_links` (JSON key /
  MEX `options.rigid_links`, node pairs) merge into rigid bodies solved master–slave (RBE2); a BC on a
  slave DOF becomes a Lagrange row (rank-revealing QR fallback for redundant pins); mechanisms raise an
  error. Same logic for standalone/server/MEX. V2 suspension mounts attach to the nearest structural node
  by a rigid tab within `config.mount_tab_max_m` (75 mm) or by one support tube otherwise
  (`elements_geom.rigid_links`); the hard-coded 50 mm bracket tubes are gone (bracket nodes remain unused).
  Not yet: engine as RBE2 stressed member (follow-up).

---

## 参考文献

* **FSAE Rules**: Formula SAE 2026 V1.0 (T.1.1, F.2, F.3)
* **CIDECT**: "Design Guide for Circular Hollow Section (CHS) Joints" (1991)
* **Eigen3**: [http://eigen.tuxfamily.org/](http://eigen.tuxfamily.org/)

---

**作成者**: Claude Sonnet 4.5

**プロジェクト**: Formula TMU FSAE Team

**日付**: 2026-02-24
