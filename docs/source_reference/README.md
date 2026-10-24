# ソースリファレンス

ここでは、**実コードのパス構造に合わせて 1 ファイル 1 Markdown** で説明を配置します。

## 方針

- 現在使っている自作コードだけを対象にする
- 生成物、外部データ、ベンダー配布物は対象外にする
- 初学者でも追えるよう、役割・入出力・主要関数・理論背景をまとめる
- 入口が変わったら、この索引と個別文書を同時更新する

## 主要ディレクトリ

- [cpp/](cpp/README.md) — 14DOF、ラップタイム、物理モデル、Google Test
- [matlab/](matlab/README.md) — EasyStart 下流、統合実行、RL、POU
- [suspension_kinematics_design/](suspension_kinematics_design/README.md) — フロント/リア A アーム最適化
- [frame_optimize/](frame_optimize/README.md) — フレーム最適化と FEM 連携
- [tire_fitting/](tire_fitting/README.md) — Python ベースの物理モデル同定
- [tire_pinn/](tire_pinn/README.md) — PINN 学習と運用
- [python/](python/README.md) — パッケージと可視化
- [dashboard/](dashboard/README.md) — FastAPI UI
- [scripts/](scripts/README.md) — シェル / 解析補助
- [tests/](tests/README.md) — pytest と統合検証

## ルート直下ファイル

ルートに置く説明対象は、本当に入口であるものに絞ります。

- `EasyStart.m`
- `build_easystart_docker_environment.m`
- `resolve_easystart_docker_build_profile.m`
- `profile_rl_bridge.m`
- `docker-entrypoint.sh`
- `setup_env.sh`

解析スクリプトや単発テストは、それぞれ移設先のディレクトリ配下で説明します。

## 読み順

1. 対象ディレクトリの `README.md`
2. 個別ファイルの `*.md`
3. 背景理論が必要なら `../../学生フォーミュラ車両運動シミュレーション総合ガイド.md`
