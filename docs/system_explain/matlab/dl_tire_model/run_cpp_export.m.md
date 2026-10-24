# `matlab/dl_tire_model/run_cpp_export.m`

## 1. 概要

学習済みモデルを C++ 向けにエクスポートするスクリプトです。格子点設定を明示し、`exportForCpp` を呼び出します。

## 2. ファイルの役割

`output/ultra_deep_validation_model.mat` を入力モデルとして固定し、C++ 側で使うための出力ディレクトリ `cpp_export` と格子設定を定義する実行スクリプトです。総格子点数と推定メモリ使用量を表示してから、実際のエクスポート処理を委譲します。

## 3. インターフェース

- 種別: スクリプト
- 主要変数:
  - `model_path = 'output/ultra_deep_validation_model.mat'`
  - `output_dir = 'cpp_export'`
  - `grid_config`
    - `SR_range = [-0.2, 0.2, 40]`
    - `SA_range = [-6, 6, 50]`
    - `Fz_range = [200, 1300, 30]`
    - `IA_range = [0, 4, 15]`
    - `TSTC_range = [55, 80, 15]`
  - `total_points`
- 出力:
  - 画面ログ
  - `exportForCpp` が生成するファイル群

## 4. 依存関係

- 同ディレクトリ内の関数:
  - `exportForCpp`
- MATLAB 組み込み:
  - `clear`
  - `clc`
  - `fprintf`
  - `isfile`
  - `error`
  - `struct`

## 5. 処理の流れ

1. `clear; clc;` で状態を初期化します。
2. ヘッダを表示します。
3. モデルファイルと出力先ディレクトリを固定文字列で設定します。
4. `isfile(model_path)` でモデル存在を確認し、なければ `error` で停止します。
5. 5 次元格子の範囲と分割数を `grid_config` に設定します。
6. 各軸の点数積から `total_points` を求めます。
7. 総点数と `total_points * 4 * 8 / 1e6` に基づく推定メモリを表示します。
8. `exportForCpp(model_path, output_dir, grid_config)` を実行します。
9. 完了メッセージを表示します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- セクション分割はありません。
- 実処理本体は `exportForCpp` に委譲し、本ファイルは入力固定と格子設定の定義に集中しています。

### ローカル関数

- ありません。

## 7. 注意点

- モデル入力は `ultra_deep_validation_model.mat` に固定されています。
- 推定メモリ計算は「格子点数 × 4 出力 × 8 バイト」を前提にしており、補助データは含みません。
- 出力先は `run_export_for_cpp.m` と異なり `cpp_export` です。
