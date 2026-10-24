# 現行システムマップ

この文書は、**今どの入口が正で、どの順で読めば迷いにくいか**をまとめた運用地図です。

## 1. 標準導線

| やりたいこと | 入口 | 下流で使う主ファイル |
|:---|:---|:---|
| 日常操作を始める | `EasyStart.m` | `matlab/run_full_pipeline.m` |
| 車両諸元GAを回す | `EasyStart.m` メニュー [3] | `matlab/run_full_vehicle_ga.m`, `docs/vehicle_ga_platform.md` |
| 14DOF を直接追う | `cpp/src/main_14dof.cpp` | `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp` |
| ラップタイムを追う | `cpp/src/main_laptime_sim.cpp` | `track/*`, `velocity_profile.hpp`, PID / RL / `lpv_lqr` / `mpc` / `mpc_v2` / `attack` driver |
| 設計感度分析を見る | `EasyStart.m` メニュー [4] | `matlab/design_sensitivity/run_design_sensitivity.m` |
| タイヤ運用を見る | `EasyStart.m` メニュー [6] | `tire_pinn/`, `tire_fitting/`, `nn_lookup_tire.hpp` |
| サスペンション設計を見る | `EasyStart.m` メニュー [8] | `suspension_kinematics_design/*.m` |
| フレーム最適化を見る | `EasyStart.m` メニュー [9] | `frame_optimize/examples/run_canonical_frame_optimization.m`, `frame_optimize/matlab/frameObjectiveFunction.m` |
| RL を見る | `EasyStart.m` メニュー [10] | `matlab/rl_driver/*`, `cpp/src/fsae_rl_server.cpp` |
| AC MOD 書き出しを見る | `EasyStart.m` メニュー [11] | `matlab/export_assetto_corsa_nk16.m`, `matlab/install_assetto_corsa_nk16_export.m` |
| 理論・実装の修正履歴を見る | `docs/theory_and_implementation_fixes.md` | Magic Formula 符号、RK4副作用、ドライバー連動、フレームねじり、fail-fast |

## 2. 正本として扱う実装

### 14DOF

- 正本: `cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`
- 実行入口: `cpp/src/main_14dof.cpp`
- 状態数: 44
- 14DOF デモCSVは各輪 `Fz` と lagged slip angle を出力し、ゼロ埋め検証列は使いません

### タイヤ

- 運用上の主系統: PINN
- C++ 実装の候補: `nn_lookup_tire.hpp`, `magic_formula.hpp`, `tmeasy.hpp`
- 同定・比較: `tire_fitting/`
- Magic Formula は TTC 符号を内部保持し、14DOF 接続時に `Fx` のみ車両側規約へ変換します
- `tire_fitting/main.py` は互換入口で、実体は Hybrid 対応の `tire_fitting/main_hybrid.py` です

### 最適化

- 車両諸元GA: `matlab/run_full_vehicle_ga.m`（Autocross / LPV-LQR steering6 / PINN固定）
- 車両GAの設定SSOT: `matlab/vehicle_ga_default_config.m`
- 車両GAの出力: `output/vehicle_ga_runs/<run_id>/`。受入済み最新解だけ`latest_best_vehicle_artifact.json`へ昇格
- サスペンション: `suspension_kinematics_design/`
- POU: `matlab/pou_hardpoint_design/`
- フレーム: `frame_optimize/examples/run_canonical_frame_optimization.m` → `frame_optimize/matlab/frameObjectiveFunction.m`
- `frame_optimize/examples/run_frame_optimization.m` と `frame_optimize/matlab/runFrameOptimization.m` は旧デモ互換ラッパーであり、正規入口ではありません
- FEM ねじり剛性は分布 `uz` カップル荷重全体からトルクと代表ねじれ角を算出します
- POU は artifact 由来の多項式・ハードポイントが不足する場合は fail-fast します
- サスペンション逆解析ターゲットは `targets.sources` と `targets.model_scope` に由来と仮定を残します
- フレーム目的関数は FEM 結果に加えて `checkFrameCompliance()` の penalty/report を目的関数へ統合します

### Docker 検証

- 標準イメージ: `docker compose build fsae-sim`
- 全検証: `docker compose run --rm fsae-sim test`
- 個別検証: `docker compose run --rm fsae-sim test-cpp` / `docker compose run --rm fsae-sim test-python`
- entrypoint はテスト前に必ず CMake configure/build を走らせるため、古い build cache の実行漏れを避けます

### 感度分析

- 入口: `EasyStart.m` メニュー [4] → `matlab/design_sensitivity/run_design_sensitivity.m`
- 最小OAT: `build_full_vehicle_design_schema` の46変数を autocross + LPV/PID で感度評価する。
- baseline は必須ゲート。baseline が完走しない場合は sweep を開始しない。
- 出力: `output/runtime/design_sensitivity/<run_id>/` の `samples.csv` / `summary.csv` / `freeze_candidates.txt` / `manifest.json`
- 旧 `matlab/sensitivity` は削除済み。46D全変数・grouped Morris・GA接続は Issue #171 で継続する。

### ドライバー

- 本線: LPV-LQR（`--driver lpv_lqr`、有限フレーム＋トルクモード）
- PID: 安定フォールバック・教師・診断用
- MPC / MPC v2: 非推奨のレガシー。性能改善・新規採用は行わない。
- Issue #160 / #164（MPC系）は not planned でクローズ。

### 車両諸元 JSON

- 正規の集約 / 保存 API: `matlab/build_vehicle_artifact.m` / `matlab/save_vehicle_artifact.m`
- 正規の読込 API: `matlab/load_vehicle_artifact.m`
- 運用上の主要成果物: `matlab/optimized_*.json`
- サスペンション逆解析の正本:
  - runtime 消費データ: `params.suspension.kc.front|rear|corners`
  - 設計メタデータ: `params.suspension_geometry`
  - 更新 API: `matlab/save_suspension_geometry_artifact.m`

## 3. 初学者向けの読み順

1. `README.md`
2. `学生フォーミュラ車両運動理論総合ガイド.md`
3. `学生フォーミュラ車両運動シミュレーション総合ガイド.md`
4. `EasyStart.m`
5. 興味のある主入口
6. `docs/source_reference/` の個別ファイル説明

## 4. サブシステムの役割

### MATLAB

- 運用のハブ
- EasyStart とパイプライン制御
- 設計最適化
- RL 学習

### C++

- 車両運動とラップタイムの本体
- 1 ms RK4 ベース
- 14DOF / 3DOF / 6DOF / 8DOF / RL サーバ

### Python

- タイヤ同定
- 結果可視化
- 補助解析

### Dashboard

- パラメータ編集と可視化の入口
- 将来的な Web 中心運用の土台

## 5. 出力先の標準

- `output/runtime/`
- `output/laptime/`
- `output/reports/`
- `output/14dof/`
- `output/quarter_car/`
- `output/assetto_corsa/`
- `output/vehicle_ga_runs/`

ルート直下の CSV / JSON は標準導線として扱いません。

## 6. 単一 JSON 管理の現状

- **サスペンション / 空力 / LSD / タイヤ / POU 係数 / `K_chassis`** は、`optimized_*.json` 系の vehicle artifact に集約
- **サスペンション hardpoints / KC surrogate / compliance surrogate** も `params.suspension.kc.*` と `params.suspension_geometry` として同じ artifact に保存
- **POU 物理ハードポイント** は `matlab/pou_hardpoint_design/export/update_vehicle_params_json.m` により同じ JSON へ追記
- **フレーム最適化の詳細結果** は `frame_optimize/examples/run_frame_optimization.m` から `matlab/save_frame_optimization_artifact.m` を介して同じ vehicle artifact JSON に保存
- 下流の `matlab/run_full_pipeline.m` は `opt.custom_params_file` でこの artifact を明示指定して利用する（暗黙の自動選択はしない）
- フレーム最適化は、サスペンション hardpoints を含む既存 vehicle artifact を入力側の SSOT として解決してから実行する
- **フレーム最適化 V2** は `frame_design_schema_v2()` / `decode_frame_design_vector()` で設計変数を一元管理し、`UseFrameDesignV2=true` のとき FBH / FH / MH / MHB / リア終端台形を GA 変数として扱う
- V2 は `params.frame.diff_mount_points` または `params.powertrain.diff_mount_points` を必須入力にし、未指定時は仮値で継続せず fail-fast する
- V2 の `params.frame` は `frame_result/v2` として、設計 schema、optimizer mode / seed、solver backend、FEM lifecycle、目的関数係数、全荷重ケース結果、規定チェックレポートを保存する
- 15000 評価規模の MATLAB `ga` 計測と最終PNG生成は `frame_optimize/examples/run_frame_v2_targeted_ga_budget.m` を使う。探索中は `torsion_only`、最終個体は全荷重ケースで再評価する。現在の調査用既定は MEX backend を優先し、MEX がある場合は Docker/TCP FEM server を起動しない
- 2026-05-10 の安定化後スモークでは、309 funccount、61.2 s、0.1980 s/eval、finite score rate 100%、true valid individual rate 62.5%、45.06 kg、4909 Nm/deg。MEX 単体は同フレームで median 0.001432 s/eval なので、残る速度支配因子は MATLAB 目的関数/規定チェック/GA overhead 側
- 2026-05-11 時点で `frame_optimize/examples/experiment_frame_v2_mass_cap_stiffness.m` は `TopologyMode='artifact'|'regulation_minimal'` を持ち、固定 97 要素 topology と規定最小寄り topology の質量下限を比較できる。diff mount あり sparse probe は 27.23 kg / 3357 Nm/deg、suspension-only sparse probe は 24.28 kg / 3357 Nm/deg
- 上記 sparse probe は現 V2 node set 上の下限確認であり、Upper Side Impact の F.6.4 幾何はまだ完全ではない

現在は、ラップタイムシミュレータで使う車両諸元に加え、  
フレーム最適化の topology / pipe selection / nodes / connectivity / 性能指標も単一 JSON に統合される運用になっています。
