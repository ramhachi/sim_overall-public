# `build_easystart_docker_environment.m`

> **要約**: Docker コンテナの起動・ビルド・コンパイルを自動化する MATLAB 関数。`EasyStart` の環境構築メニューから呼ばれる。

## ファイルの目的

Docker Desktop の起動確認、コンテナの `docker compose up --build`、C++ コードの CMake + Ninja ビルドを一連の手順として自動実行する。ホスト OS のアーキテクチャを検出し、最適なビルドプロファイルを選択する。

## 主要関数

| 関数 | 説明 |
|:---|:---|
| `build_easystart_docker_environment()` | メイン関数。Docker 起動→コンテナビルド→C++ コンパイルを実行 |
| `wait_for_service_ready(service_name, max_attempts)` | コンテナの準備完了をポーリングで待機 |
| `restore_environment(original_dir, previous_docker_platform)` | 処理完了後に作業ディレクトリと環境変数を復元 |
| `select_host_port(env_name, preferred_port)` | 環境変数または空きポートから使用ポートを決定 |
| `find_available_tcp_port(preferred_port)` | Java ソケットを使って空き TCP ポートを探索 |
| `can_bind_tcp_port(port)` | 指定ポートがバインド可能か確認 |

## 理論的背景

クロスプラットフォーム対応のため、ホスト CPU アーキテクチャ（ARM64/AMD64）を検出し、Docker プラットフォーム指定と CMake 最適化プロファイルを自動選択する。`resolve_easystart_docker_build_profile.m` が返すプロファイルに基づいてビルドを実行する。

## ビルド構成

- **Docker プラットフォーム**: `linux/arm64` or `linux/amd64`（自動検出）
- **CMake ジェネレータ**: Ninja
- **ビルドターゲット**: `laptime_sim`, `test_14dof_new`, `fsae_rl_server` 等
- **ポート割り当て**: UI (8080), RL (22345), FEM (50051)

## 使用する MATLAB 機能

| 機能 | 目的 |
|:---|:---|
| `system()` | Docker CLI コマンドの実行 |
| `onCleanup` | 異常終了時の環境復元（RAII パターン） |
| `java.net.ServerSocket` | 空き TCP ポートの動的割り当て |
| `setenv` / `getenv` | 環境変数操作（DOCKER_DEFAULT_PLATFORM 等） |

## 入出力

- **入力**: なし（内部でアーキテクチャとポートを自動検出）
- **出力**: `success` (logical) — ビルド成功なら `true`
- **副作用**: Docker コンテナの起動、C++ バイナリのコンパイル

## システム上の役割

`EasyStart.m` のメニュー [1] から呼ばれる。初回セットアップやコード変更後のリビルドに使用。
