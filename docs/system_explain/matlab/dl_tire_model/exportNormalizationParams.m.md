# `matlab/dl_tire_model/exportNormalizationParams.m`

## 1. 概要

入力・出力の正規化パラメータをテキストへ保存する関数です。  
`INPUT_MEAN / INPUT_STD / OUTPUT_MEAN / OUTPUT_STD` の 4 セクションを順に出力します。

## 2. ファイルの役割

- MATLAB 内の `norm_params` を、外部実装でも読みやすいプレーンテキストへ変換します。
- 5 入力と 4 出力の平均値・標準偏差を固定順で保持します。

## 3. インターフェース

### 関数 `exportNormalizationParams(norm_params, output_path)`

- 入力
  - `norm_params`: `input_mean`, `input_std`, `output_mean`, `output_std` を持つ構造体
  - `output_path`: 保存先テキストファイル
- 出力
  - 戻り値はありません。
  - 指定ファイルに正規化パラメータを保存します。

## 4. 依存関係

- MATLAB 標準機能: `fopen`, `fprintf`, `datestr`, `fclose`
- 外部独自関数には依存しません。

## 5. 処理の流れ

1. 出力ファイルを開き、失敗時は `error` を送出します。
2. ヘッダとして生成日時と保存形式を記述します。
3. `INPUT_MEAN` と `INPUT_STD` を 5 要素ずつ 1 行で書き出します。
4. `OUTPUT_MEAN` と `OUTPUT_STD` を 4 要素ずつ 1 行で書き出します。
5. ファイルを閉じて完了メッセージを表示します。

## 6. 関数・ローカル関数詳細

### `exportNormalizationParams(norm_params, output_path)`

- ファイル内唯一の関数です。
- 入力は `SR, SA, Fz, IA, TSTC`、出力は `Fx, Fy, Mx, Mz` の順で固定です。
- 各数値は `%.15e` の指数表記で保存されます。

## 7. 注意点

- `norm_params` の必須フィールド存在確認は実装していません。
- 配列長の検証もないため、5 入力・4 出力の前提を呼び出し側が守る必要があります。

