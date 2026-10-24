# `resolve_easystart_docker_build_profile.m`

> **要約**: ホスト CPU アーキテクチャを検出し、Docker/CMake のビルドプロファイル（プラットフォーム・最適化・ターゲット）を決定する MATLAB 関数。

## ファイルの目的

`build_easystart_docker_environment.m` から呼ばれ、ホストマシンのアーキテクチャに応じた Docker プラットフォーム指定、CPU 最適化プロファイル、CMake ジェネレータ、ビルドターゲット一覧を構造体として返す。

## 主要関数

| 関数 | 説明 |
|:---|:---|
| `resolve_easystart_docker_build_profile()` | プロファイル構造体を返すメイン関数 |
| `detect_raw_architecture()` | OS 別にホスト CPU アーキテクチャ文字列を取得 |
| `normalize_architecture(raw_arch)` | `arm64`/`amd64` に正規化 |

## アーキテクチャマッピング

| 検出値 | 正規化 | Docker プラットフォーム | CPU プロファイル |
|:---|:---|:---|:---|
| `arm64`, `aarch64`, `maca64` | `arm64` | `linux/arm64` | `arm-native` |
| `x86_64`, `amd64`, `maci64`, `glnxa64`, `win64` | `amd64` | `linux/amd64` | `x86-native` |
| その他 | そのまま | デフォルト | `auto` |

## 返値構造体のフィールド

| フィールド | 説明 |
|:---|:---|
| `host_arch` | 正規化アーキテクチャ |
| `docker_platform` | Docker `--platform` 引数値 |
| `cpu_optimization_profile` | CMake の `-DFSAE_CPU_OPTIMIZATION_PROFILE` 値 |
| `generator` | `Ninja` |
| `build_signature` | ビルドキャッシュ識別子 |
| `build_targets` | ビルド対象バイナリ一覧 |

## ビルドターゲット

`laptime_sim`, `test_14dof_new`, `test_velocity_profile`, `test_aero_integration`, `test_aero_map_4d`, `test_lsd`, `fsae_rl_server`, `rl_actor_check`

## 入出力

- **入力**: なし（`uname -m` や `computer('arch')` で自動検出）
- **出力**: `profile` 構造体

## システム上の役割

`build_easystart_docker_environment.m` の内部ヘルパー。クロスプラットフォーム対応のビルド設定を抽象化する。
