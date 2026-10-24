# `matlab/dl_tire_model/generateAllVisualizations.m`

## 1. 概要

`generateAllVisualizations` は、モデル評価用の可視化関数をまとめて呼び出すラッパーです。散布図、残差図、時系列図、運転条件別性能図を順番に生成し、各処理の失敗は `warning` で通知して後続処理を継続します。

## 2. ファイルの役割

- 可視化出力ディレクトリを準備する
- 4 種類の可視化関数を順次実行する
- 実行完了後に `output_dir` 直下の `*.png` を列挙する

## 3. インターフェース

### 関数

`generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir, varargin)`

### 引数

- `X_features`
  - `N×5` 行列。コメントでは `[SR, SA, Fz, IA, TSTC]`。
- `Y_actual`
  - `N×4` 行列。コメントでは `[Fx, Fy, Mx, Mz]` の実測値。
- `Y_pred`
  - `N×4` 行列。コメントでは `[Fx, Fy, Mx, Mz]` の予測値。
- `output_dir`
  - 図の保存先ディレクトリ。
- `varargin`
  - `generateTimeSeriesPlots` と `generateOperatingConditionPlots` にそのまま転送される可変引数。

### 戻り値

- なし

## 4. 依存関係

- `generateScatterPlots`
- `generateResidualPlots`
- `generateTimeSeriesPlots`
- `generateOperatingConditionPlots`
- MATLAB 標準関数
  - `exist`, `mkdir`, `warning`, `dir`, `fullfile`, `fprintf`

## 5. 処理の流れ

1. 開始メッセージを表示する。
2. `output_dir` がなければ `mkdir` で作成する。
3. 散布図生成を `try/catch` で実行する。
4. 残差図生成を `try/catch` で実行する。
5. 時系列図生成を `try/catch` で実行する。
6. 運転条件別図生成を `try/catch` で実行する。
7. `dir(fullfile(output_dir, '*.png'))` で生成ファイル数とファイル名を表示する。

## 6. 関数・ローカル関数詳細

### `generateAllVisualizations`

- 4 種類の可視化をまとめて実行する便宜関数です。
- 各サブ関数の失敗は個別に補足され、識別子付き `warning` を出して次へ進みます。
- `varargin` は全サブ関数ではなく、時系列図と運転条件別図にのみ渡されます。

### ローカル関数

- 定義されていません。

## 7. 注意点

- 途中の可視化が失敗しても関数全体は停止しない実装です。
- 最後に列挙するのは `output_dir` 直下の `*.png` のみで、下位ディレクトリは対象外です。
- 入力サイズや列数の検証はこの関数自身では行わず、各可視化関数側に委ねています。
