# `EasyStart.m`

> **要約**: FSAE シミュレータの対話式ランチャー。シミュレーション実行・最適化・可視化・タイヤ管理・RL・Assetto Corsa export までの全操作を案内する。

## ファイルの目的

MATLAB コマンドウィンドウから `EasyStart` を実行するだけで、プロジェクトの主要機能にアクセスできる統合エントリポイント。Docker 環境構築、シミュレーション実行、車両パラメータ最適化、設計感度分析（最小OAT）、結果可視化、タイヤモデル学習、サスペンション設計、フレーム最適化、RL ドライバー管理、Assetto Corsa NK16 export の全 11 メニューを提供する。

## メニュー構成

| 番号 | 機能 | 説明 |
|:---|:---|:---|
| 1 | 環境構築・更新 | Docker コンテナのビルドと C++ コンパイル |
| 2 | シミュレーション実行 | トラック選択→PID パラメータ設定→ラップタイム計算 |
| 3 | 車両パラメータ最適化 | Bayesian Optimization による PID ゲイン自動探索 |
| 4 | 設計感度分析 | 46D design vector の最小OAT。baseline完了を必須ゲートにする |
| 5 | 結果の可視化 | 直近シミュレーション結果の Python プロット呼び出し |
| 6 | タイヤモデル学習 | Magic Formula / PINN の学習・一覧確認・運用用モデル生成 |
| 7 | POU ハードポイント最適化 | 多項式係数から物理的な座標を逆算 |
| 8 | サスペンション統合設計 | A-Arm + POU / Rear A-Arm の全 3D 座標一貫生成 |
| 9 | フレーム最適化 | 3D トラス＋サスペンション統合の構造最適化 |
| 10 | RL ドライバー管理 | 強化学習の訓練・評価・受け入れゲート・ace-driver 継続学習・phase4 追加学習・RL ベンチ・C++ エクスポート |
| 11 | Assetto Corsa NK16 MOD 書き出し | 完全車両 JSON から NK16 用 `data/` / `ui/` overlay を生成し、必要なら展開済み車両フォルダへ直接反映 |
| 0 | 終了 | プログラム終了 |

## 理論的背景

車両運動シミュレーションのワークフローは多段階で複雑（環境構築→パラメータ設定→シミュレーション→分析→最適化）であるため、初学者や非プログラマーのチームメンバーでも操作できるよう、対話式メニューで抽象化している。

## 技術的特徴

- **OS 別 PATH 設定**: macOS (`/opt/homebrew/bin`) と Windows (`C:\Program Files\Docker\...`) の Docker パスを自動検出・追加
- **Docker 連携**: `docker compose run` でコンテナ内のバイナリを実行
- **Python 呼び出し**: MATLAB の `system()` から Python スクリプトを実行
- **JSON プリセット選択**: `find_vehicle_json_candidates.m` / `select_vehicle_json_candidate.m` による共通候補選択
- **AC export**: `export_assetto_corsa_nk16.m` と `install_assetto_corsa_nk16_export.m` をメニュー [11] から起動
- **感度分析**: `matlab/design_sensitivity/run_design_sensitivity.m`。46D全変数・Morrisは Issue #171 で継続
- **export-ready 判定**: AC export 用候補は完全車両 JSON のみに制限
- **artifact-aware 選択**: POU / suspension / frame / AC export は共通候補選択器で車両 artifact を選ぶ
- **hardpoint 条件付き選別**: frame は `FL` / `RL` hardpoints、steering は `FL` hardpoints を前提にする
- **無限ループメニュー**: `while true` ＋ `switch-case` で継続操作
- **RL ace 継続導線**: recovery 完走 agent から速度寄り mixed-track continuation を起動可能

## 使用する外部ツール・スクリプト

| 呼び出し先 | 目的 |
|:---|:---|
| `build_easystart_docker_environment.m` | Docker 環境構築 |
| `docker compose run ... laptime_sim` | C++ シミュレーション実行 |
| `plot_autocross_result.py` | Python 可視化 |
| `tire_fitting/main.py` | タイヤパラメータフィッティング |
| `validate_all_pinn_models.py` | PINN モデル検証 |
| `export_assetto_corsa_nk16.m` | NK16 向け AC overlay 生成 |
| `install_assetto_corsa_nk16_export.m` | 展開済み AC 車両フォルダへの直接反映 |

## 入出力

- **入力**: ユーザーのメニュー選択（標準入力）
- **出力**: 各機能の実行結果（シミュレーション CSV、プロット PNG 等）

## システム上の役割

プロジェクト全体のフロントエンド。MATLAB ユーザーにとってのメインエントリポイントであり、すべての機能へのゲートウェイとして機能する。

## 実行方法

```matlab
>> EasyStart
```
