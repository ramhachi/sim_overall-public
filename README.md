# FSAE Vehicle Dynamics Simulator

このリポジトリの標準入口は **`EasyStart.m`** です。  
日常運用では MATLAB からメニューを開き、シミュレーション、最適化、タイヤ学習、サスペンション設計、フレーム最適化、RL ドライバー管理を順番に操作します。

## このシステムがしていること

- **C++**: `vehicle_14dof_new.hpp` を中核にした 14DOF 車両運動計算、ラップタイム評価、RL サーバ
- **MATLAB**: `EasyStart.m` と `matlab/run_full_pipeline.m` を中心にした実行オーケストレーション
- **PINN タイヤ**: 運用上の主系統。利用可能な学習済みモデルがあれば EasyStart から優先選択
- **物理モデルタイヤ**: `Magic Formula` / `TMeasy` は比較・フォールバック用
- **設計最適化**: サスペンション幾何、POU ハードポイント、フレーム構造
- **Python**: タイヤ同定、可視化、補助解析

## 最初に使うファイル

| 目的 | 入口 |
|:---|:---|
| 理論を読む | `学生フォーミュラ車両運動理論総合ガイド.md` |
| 実装全体像を読む | `学生フォーミュラ車両運動シミュレーション総合ガイド.md` |
| 通常操作を始める | `EasyStart.m` |
| いま動いている構成を見る | `docs/current_system_map.md` |
| LPV車両GAを保守する | `docs/vehicle_ga_platform.md` |
| 最近修正した理論・実装の整合性を見る | `docs/theory_and_implementation_fixes.md` |
| ファイル別説明を読む | `docs/source_reference/README.md` |

## EasyStart でできること

`EasyStart.m` のメニューは次の 11 系統です。

1. Docker 環境構築・更新
2. 単走シミュレーション
3. 車両パラメータ最適化
4. 設計感度分析（最小OAT）
5. 結果可視化
6. タイヤモデル学習 / フィッティング
7. POU ハードポイント最適化
8. サスペンション統合設計
9. フレーム最適化
10. RL ドライバー管理
11. Assetto Corsa NK16 MOD 書き出し

CLI は保守や検証には有用ですが、通常の利用導線としては EasyStart を優先します。

## 現在の主系統

### 14DOF 車両

- 正式な実装は **`cpp/include/fsae/vehicle/vehicle_14dof_new.hpp`**
- 実行入口は `cpp/src/main_14dof.cpp`
- 状態数は内部状態込みで **44**
- 代表ケースは `step_steer`, `circle`, `acceleration`, `cornering`

### タイヤ

- C++ 側は `NN -> Magic Formula -> TMeasy` の順で利用可能性を判定
- 運用上は **PINN を主系統**として扱う
- `tire_fitting/` は物理モデル寄りの同定や比較用パラメータ生成を担う
- EasyStart は deployable な PINN モデルがある場合にそれを既定候補として提示

### 最適化

- **車両諸元GA**: `EasyStart.m` メニュー `[3]` から、LPV-LQR・PINN固定の再開可能な5時間探索を実行
- **サスペンション**: `suspension_kinematics_design/` がフロント/リア A アームを生成
- **POU**: `matlab/pou_hardpoint_design/` が多項式表現から物理ハードポイントを復元
- **フレーム**: `frame_optimize/` がサスペンション取付点を取り込んで GA + FEM で最適化

POU 最適化は物理ハードポイントが下流の根拠になるため、`matlab/run_full_pipeline.m` では
失敗時にデフォルトで停止します。診断目的で続行する場合だけ
`'allow_pou_failure', true` を明示してください。

## 出力先

ルート直下に生成物を散らさないよう、主な出力は `output/` に集約します。

- `output/runtime/` — 実行時 JSON
- `output/laptime/` — ラップタイム CSV
- `output/reports/` — レポート
- `output/14dof/` — 14DOF 単体出力
- `output/quarter_car/` — quarter car 検証出力
- `output/vehicle_ga_runs/` — 個体ログ、checkpoint、最終車両artifact

## ディレクトリの見方

```text
sim_overall/
├── EasyStart.m
├── 学生フォーミュラ車両運動理論総合ガイド.md
├── 学生フォーミュラ車両運動シミュレーション総合ガイド.md
├── cpp/                           # 14DOF、ラップタイム、物理モデル、テスト
├── matlab/                        # 統合実行、RL、POU、各種最適化補助、parameter_presets
├── suspension_kinematics_design/  # Aアーム幾何最適化
├── frame_optimize/                # フレーム最適化と FEM 連携
├── tire_fitting/                  # Python タイヤ同定
├── tire_pinn/                     # PINN 学習・運用関連
├── python/                        # 可視化と補助パッケージ
├── dashboard/                     # FastAPI ベース UI
├── scripts/                       # 補助スクリプト
├── tests/                         # pytest
├── docs/                          # 現役文書とソース別リファレンス
└── output/                        # 生成物
```

## コマンドラインで保守したいとき

### Docker

他メンバ配布とCI相当の検証は Docker 経由を標準にします。C++/Python/PINN 用の依存関係は
`Dockerfile` に集約しています。

```bash
docker compose build fsae-sim
docker compose run --rm fsae-sim build
docker compose run --rm fsae-sim test-cpp
docker compose run --rm fsae-sim test-python
docker compose run --rm fsae-sim test
```

`test` は CMake configure/build、C++ `ctest`、Python `pytest tests/ -v` を順に実行します。

### C++

```bash
mkdir -p build && cd build
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build .
ctest --output-on-failure
```

### 代表実行

```bash
./cpp/fsae_sim
./cpp/fsae_14dof step_steer
./cpp/fsae_14dof circle
./cpp/fsae_14dof acceleration
./cpp/fsae_14dof cornering
```

### Python

```bash
pytest tests/ -v

python3 tire_fitting/main.py \
    --drivebrake data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv \
    --cornering data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv \
    --output output/tire_params \
    --tire-name Hoosier_18x6_R20
```

### MATLAB / MEX

フレーム MEX FEM は classic C MEX API の `mexFunction()` を C++ で実装しています。
`frame_optimize/matlab/build_mex.m` は classic ABI の `-R2017b` を使い、macOS の
MATLAB R2025b では C++ MEX adapter export を `LINKEXPORTCPP=` で無効化します。

```bash
bash scripts/run_matlab_batch.sh "addpath(fullfile(pwd,'frame_optimize','matlab')); build_mex();"
```

## ドキュメント構成

- `学生フォーミュラ車両運動シミュレーション総合ガイド.md`  
  実装、全体構成、運用対応
- `学生フォーミュラ車両運動理論総合ガイド.md`  
  数学・物理の理論体系
- `docs/current_system_map.md`  
  現役の入口と標準フロー
- `docs/source_reference/`  
  1 ファイル 1 Markdown の実装説明
- `docs/future_roadmap.md`  
  未実装項目と今後の整理方針

細部は `docs/source_reference/README.md` から追ってください。
