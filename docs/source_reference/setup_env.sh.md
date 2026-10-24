# `setup_env.sh`

> **要約**: FSAE シミュレータのネイティブ開発環境を一括セットアップするシェルスクリプト（vcpkg・依存関係・Python・C++ ビルド・テスト）。

## ファイルの目的

Docker を使わずにホストマシン上で直接ビルド・テストを行うための開発環境を構築する。vcpkg のセットアップ、C++ 依存関係のインストール、Python 仮想環境の構築、CMake ビルド、テスト実行を段階的に実行する。

## 理論的背景

Docker 環境（`EasyStart.m` → `build_easystart_docker_environment.m`）とは別に、ホスト上での直接ビルドが必要なケース（IDE デバッグ、CI/CD、Docker 非対応環境）に対応する。このリポジトリは C++ / Python / MATLAB / Docker が絡むため、環境差分が出やすく、セットアップスクリプトで手順を固定化している。

## 主要関数

| 関数 | 説明 |
|:---|:---|
| `detect_triplet()` | OS + CPU アーキテクチャから vcpkg トリプレット（`arm64-osx` 等）を決定 |
| `cpu_count()` | 利用可能な CPU コア数を取得（ビルド並列度に使用） |
| `persist_vcpkg_root()` | `VCPKG_ROOT` をシェル設定ファイルに永続化 |
| `configure_cmake_project()` | CMake の構成コマンドを vcpkg ツールチェーン付きで実行 |
| `setup_vcpkg()` | vcpkg のクローンとブートストラップ |
| `install_dependencies()` | vcpkg マニフェストに基づく依存関係インストール |
| `setup_python()` | Python 仮想環境の作成と `pip install -e ".[dev]"` |
| `build_cpp()` | コアプロジェクトとフレーム最適化 FEM バイナリのビルド |
| `run_tests()` | CTest によるテスト実行 |
| `main()` | サブコマンド分岐（`vcpkg`, `deps`, `python`, `build`, `test`, `all`） |

## vcpkg トリプレットマッピング

| OS | アーキテクチャ | トリプレット |
|:---|:---|:---|
| Darwin | arm64 | `arm64-osx` |
| Darwin | x86_64 | `x64-osx` |
| Linux | aarch64 | `arm64-linux` |
| Linux | x86_64 | `x64-linux` |

## 入出力

- **入力**: サブコマンド引数（`vcpkg`, `deps`, `python`, `build`, `test`, `all`）
- **出力**: ビルド成果物（`build/release/`）、Python 仮想環境（`python/.venv/`）

## 実行方法

```bash
./setup_env.sh          # 全ステップ実行
./setup_env.sh build    # C++ ビルドのみ
./setup_env.sh python   # Python 環境のみ
```

## システム上の役割

Docker を使わないネイティブ開発フロー向けの一括セットアップ。CI/CD パイプラインでも利用可能。
