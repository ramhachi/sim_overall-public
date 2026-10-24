# `setup_env.bat`

## 役割

- Windows 向けの**開発環境セットアップ兼ビルドスクリプト**です。
- vcpkg 準備、Python 仮想環境構築、出力ディレクトリ作成、本体 C++ と `frame_optimize/cpp` のビルドまでを一括実行します。

## 運用上の入口

- `setup_env.bat` を直接実行します。
- 対話引数はなく、リポジトリ直下基準で固定手順を進めます。

## 主要処理

### 1. vcpkg 準備
- `VCPKG_ROOT` が未設定なら `%USERPROFILE%\\vcpkg` を探し、なければ clone と bootstrap を実行します。
- `setx` で `VCPKG_ROOT` を永続化します。

### 2. vcpkg 依存導入
- `vcpkg.exe install --triplet=x64-windows --x-manifest-root="%~dp0"` を実行し、manifest 依存を導入します。

### 3. Python 環境構築
- `python\\.venv` がなければ `python -m venv .venv` を作成します。
- 仮想環境を有効化し、`pip install -e ".[dev]"` を実行します。

### 4. 出力ディレクトリ作成
- `output`, `data\\ttc`, `data\\aero`, `data\\vehicle` を不足時に作成します。

### 5. CMake ビルド
- 本体を `cmake --preset=release` と `cmake --build build/release --config Release` でビルドします。
- 続けて `frame_optimize\\cpp` を明示的に configure/build/install します。

## 入出力・副作用

- **入力**: Windows 環境、`git`, `python`, `cmake`, `vcpkg`、リポジトリ manifest
- **出力**:
  - `%USERPROFILE%\\vcpkg`
  - `python\\.venv`
  - `build\\release`
  - `frame_optimize\\cpp\\build`
- **副作用**:
  - `VCPKG_ROOT` を永続化
  - 依存導入とビルド成果物生成
  - 実行末尾で `pause`

## データフロー

1. `VCPKG_ROOT` 解決と永続化
2. vcpkg 依存導入
3. Python 仮想環境構築
4. 出力ディレクトリ初期化
5. 本体 C++ ビルド
6. `frame_optimize/cpp` ビルドと install
7. 次アクションを表示して終了

## システム内での位置づけ

- `setup_env.sh` の Windows 版に相当する**オンボーディング入口**です。
- EasyStart や Docker を使わず、ローカル Windows で C++/Python/フレーム最適化側の実行基盤を揃える役割を担います。
