# `setup_env.sh`

## 1. 役割

リポジトリ全体の **開発環境セットアップ兼ビルド補助スクリプト** です。  
vcpkg 準備、依存導入、Python 仮想環境作成、C++ ビルド、テスト実行をサブコマンド単位でまとめています。

## 2. 実行と入出力

- 実行例:
  - `./setup_env.sh`
  - `./setup_env.sh python`
  - `./setup_env.sh build`
- 主な入力:
  - ホスト OS/CPU 情報
  - `VCPKG_ROOT`
  - `python/pyproject` 相当の Python パッケージ設定
  - CMake プロジェクト
- 主な出力:
  - `$HOME/vcpkg`
  - `python/.venv`
  - `build/release`
  - `frame_optimize/cpp/build`

## 3. 主要制御フロー

1. `main()` が `vcpkg|deps|python|build|test|all` を受け取ります。
2. `detect_triplet()` で OS/CPU に応じた vcpkg triplet を選びます。
3. `setup_vcpkg()` が `VCPKG_ROOT` を用意し、必要なら clone/bootstrap します。
4. `install_dependencies()` が manifest モードで依存を導入します。
5. `setup_python()` が `python/.venv` を作り、`pip install -e ".[dev]"` を実行します。
6. `build_cpp()` が本体と frame_optimize 側 C++ を別々に configure/build/install します。
7. `run_tests()` が `build/release` で `ctest --output-on-failure` を実行します。

## 4. 関数

### `info() / warn() / error()`
- 役割: 色付きログ出力。
- `error()` はメッセージ出力後に即終了します。

### `detect_triplet()`
- 役割: `uname -s` と `uname -m` から `arm64-osx`, `x64-osx`, `arm64-linux`, `x64-linux` を返します。
- 副作用: 未対応環境では `error()` により終了。

### `cpu_count()`
- 役割: 並列ビルド用 CPU 数を取得します。
- 優先順位: `nproc` → `sysctl -n hw.ncpu` → 既定値 4

### `persist_vcpkg_root()`
- 役割: `VCPKG_ROOT` を `~/.zshrc` または `~/.bashrc` に追記します。
- 副作用: シェル設定ファイルを更新します。

### `configure_cmake_project(source_dir, build_dir, triplet, manifest_dir)`
- 役割: 共通 CMake configure ラッパです。
- 副作用: 指定ビルドディレクトリに CMake キャッシュを生成します。

### `setup_vcpkg()`
- 役割: vcpkg の存在確認、clone、bootstrap、環境変数設定を行います。

### `install_dependencies()`
- 役割: 現在リポジトリの manifest を使って vcpkg 依存を導入します。

### `setup_python()`
- 役割: `python/.venv` を構築し、開発依存込みで editable install します。
- 副作用: `source .venv/bin/activate` を内部で実行します。

### `build_cpp()`
- 役割: 本体 C++ と `frame_optimize/cpp` の両方を Release ビルドします。
- 副作用:
  - `frame_optimize/bin` へインストール
  - ビルド成果物生成

### `run_tests()`
- 役割: Release ビルド済みディレクトリで `ctest` を実行します。

### `main()`
- 役割: サブコマンド分岐です。
- `all` は vcpkg → deps → python → build → test を順に実行します。
