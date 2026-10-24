# `matlab/dl_tire_model/run_final_validation.m`

## 1. 概要

最終システム検証を起動し、結果を保存する実行スクリプトです。

## 2. ファイルの役割

Deep Learning Toolbox と Statistics and Machine Learning Toolbox のライセンス確認、GPU 有無の表示、`final_system_validation()` の呼び出し、結果保存、例外時のスタック表示をまとめて担当します。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `required_toolboxes`
  - `missing_toolboxes`
  - `results`
  - `ME`
- 出力:
  - 画面ログ
  - `output/final_validation_results.mat`

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `final_system_validation`
- MATLAB 組み込み:
  - `clear`
  - `clc`
  - `addpath`
  - `pwd`
  - `fprintf`
  - `license`
  - `gpuDeviceCount`
  - `gpuDevice`
  - `save`
  - `rethrow`

## 5. 処理の流れ

1. `clear; clc;` で初期化します。
2. `addpath(pwd)` で現在ディレクトリをパスへ追加します。
3. バナーを表示します。
4. `required_toolboxes` の 2 行 2 列セル配列を走査し、`license('test', toolbox_id)` で利用可否を確認します。
5. 欠落ライセンスがあれば警告一覧を表示し、なければ正常メッセージを出します。
6. `gpuDeviceCount` を用いて GPU 有無を表示します。
7. `try` 内で `results = final_system_validation();` を実行します。
8. 成功時は `save('output/final_validation_results.mat', 'results')` を実行します。
9. 失敗時は `ME.message` と `ME.stack` を行番号付きで表示し、`rethrow(ME)` します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- セクション分けはありません。
- ツールボックス確認と本検証実行を 1 本のフローにまとめています。
- `missing_toolboxes` はライセンス未取得時だけ追加され、処理自体は継続します。

### ローカル関数

- ありません。

## 7. 注意点

- 必須ツールボックス欠落時も即終了せず、「一部テスト失敗の可能性」を警告するだけです。
- 保存先 `output` ディレクトリの存在確認はしていません。
- 実際の検証内容は `final_system_validation()` 側にあり、本ファイルは起動・監視用です。
