# `matlab/dl_tire_model/exportModel.m`

## 1. 概要

学習済みネットワークと正規化パラメータを `.mat` ファイルへ保存する関数です。  
保存前の引数検証、出力ディレクトリ作成、保存後の存在確認まで行います。

## 2. ファイルの役割

- 学習成果物を後段の推論やエクスポート処理で再利用できる形に保存します。
- `.mat` には `trained_net` と `norm_params` をそのまま格納します。

## 3. インターフェース

### 関数 `exportModel(trained_net, norm_params, output_path)`

- 入力
  - `trained_net`: 学習済みネットワーク
  - `norm_params`: `input_mean`, `input_std`, `output_mean`, `output_std` を含む構造体
  - `output_path`: 保存先 `.mat` パス
- 出力
  - 戻り値はありません。
  - 指定パスに `.mat` ファイルを生成します。

## 4. 依存関係

- MATLAB 標準機能: `nargin`, `isfield`, `fileparts`, `exist`, `mkdir`, `save`, `isfile`, `dir`, `fprintf`
- 外部独自関数には依存しません。

## 5. 処理の流れ

1. 引数数が 3 未満なら `error` で中断します。
2. `norm_params` に必要 4 フィールドがあるか順に確認します。
3. `output_path` の親ディレクトリが存在しなければ作成します。
4. `save(output_path, 'trained_net', 'norm_params', '-v7.3')` で保存します。
5. 保存後に `isfile` で存在確認し、失敗時は `error` を出します。
6. `dir` でファイルサイズを取り、MB 単位で表示します。

## 6. 関数・ローカル関数詳細

### `exportModel(trained_net, norm_params, output_path)`

- ファイル内唯一の関数です。
- 保存形式は明示的に `-v7.3` を指定しています。
- 生成物の検証は「ファイルが存在するか」までで、内容の再読込検証は行いません。

## 7. 注意点

- `norm_params` の各フィールド値のサイズや数値妥当性は確認していません。
- 既存ファイルがあれば `save` により上書きされます。
- ネットワークの再現性確認は別ファイル `exportModelWithValidation` が担当します。

