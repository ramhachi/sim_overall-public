# `frame_optimize/cpp/CMakeLists.txt`

## 役割

- フレーム用 C++ FEM バックエンドのビルド定義です。
- スタンドアロン実行ファイル `beam_fem_standalone` と TCP サーバ `beam_fem_server` を構成します。

## 主な設定

- `cmake_minimum_required(VERSION 3.14)`
- `project(beam_fem_standalone CXX)`
- `CMAKE_CXX_STANDARD 17`

## 依存関係

- `Eigen3`
- `nlohmann_json`
- `Threads`
- `CheckCXXCompilerFlag`

## ターゲット

### `beam_fem_standalone`

- **入力**
  - `beam_fem_standalone.cpp`
- **責務**
  - JSON ファイル入出力型の FEM 実行ファイルを生成します。

### `beam_fem_server`

- **入力**
  - `beam_fem_server.cpp`
- **責務**
  - JSON over TCP で解くサーバ実行ファイルを生成します。

## `configure_fem_target(target_name)`

- **役割**
  - ターゲットごとの最適化フラグを共通設定する関数です。
- **ロジック**
  - MSVC: `/O2 /W4`
  - 非 MSVC: `-O3 -Wall -Wextra`
  - `-march=native` が使える場合のみ追加

## インストール

- `beam_fem_standalone`, `beam_fem_server` を `../bin` へ `install()` します。

## 前提・注意点

- Windows では `beam_fem_server` に `ws2_32` を追加リンクします。
- MEX ビルドはこの CMakeLists では扱っていません。
