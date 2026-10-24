# `matlab/dl_tire_model/run_export_for_cpp.m`

## 1. 概要

学習済みモデルを C++ 統合用に書き出す簡易実行スクリプトです。`exportModelForCpp` を例外処理付きで呼びます。

## 2. ファイルの役割

`run_cpp_export.m` と似た目的を持ちますが、格子設定はここでは定義せず、`exportModelForCpp` に処理を委譲しています。成功時は次の作業案内を表示し、失敗時はエラーバナーを出して再送出します。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `model_path = 'output/ultra_deep_validation_model.mat'`
  - `output_dir = '../../cpp_export'`
- 出力:
  - 画面ログ
  - `exportModelForCpp` が生成する出力物

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `exportModelForCpp`
- MATLAB 組み込み:
  - `clear`
  - `clc`
  - `fprintf`
  - `isfile`
  - `error`
  - `try` / `catch`
  - `rethrow`

## 5. 処理の流れ

1. `clear; clc;` を実行します。
2. C++ 統合向けエクスポートのヘッダを表示します。
3. モデルファイルと出力ディレクトリを設定します。
4. モデルファイルが存在しなければ `error` で停止します。
5. `try` ブロック内で `exportModelForCpp(model_path, output_dir)` を呼びます。
6. 成功時は成功バナー、出力先、次の手順 3 項目を表示します。
7. 失敗時は失敗バナーと `ME.message` を表示し、`rethrow(ME)` で例外を再送出します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 例外処理が入っているため、エクスポート関数内エラーを隠さず上位へ伝えます。
- ガイド文として「生成物確認」「C++ への統合」「検証テスト実行」を表示します。

### ローカル関数

- ありません。

## 7. 注意点

- 出力先は相対パス `../../cpp_export` です。`run_cpp_export.m` の `cpp_export` とは異なります。
- エクスポート設定の詳細はこのファイルにはなく、`exportModelForCpp` の実装依存です。
- モデル名は固定で、引数化されていません。
