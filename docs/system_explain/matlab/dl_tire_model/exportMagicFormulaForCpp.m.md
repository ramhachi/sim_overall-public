# `matlab/dl_tire_model/exportMagicFormulaForCpp.m`

## 1. 概要

Magic Formula の係数群を、C++ 側で読みやすいテキスト形式へ出力する関数です。  
`magic_formula_fitting.m` が生成した `.mat` を入力にし、セクション付きテキストを書き出します。

## 2. ファイルの役割

- `.mat` 内の `mf_params`、各段階の係数名配列、係数値配列を読み出します。
- `Fy0`, `Fx0`, `Gxa`, `Gyk`, `Mz` ごとに分けて `magic_formula_params.txt` を作成します。

## 3. インターフェース

### 公開関数 `exportMagicFormulaForCpp(mat_path)`

- 入力
  - `mat_path`: Magic Formula 係数を含む `.mat` パス。省略時は `output/magic_formula_Hoosier_18p0X6p0-10_R20.mat`
- 出力
  - 戻り値はありません。
  - `../../cpp_export/magic_formula_params.txt` を生成します。

## 4. 依存関係

- MATLAB 標準機能: `load`, `exist`, `mkdir`, `fullfile`, `fopen`, `fprintf`, `fclose`, `datestr`
- ファイル内ローカル関数: `writeSection`
- `.mat` 内に以下のフィールドがある前提です。
  - `mf_params.Fz0`, `mf_params.R0`
  - `fy_names`, `fy_params`
  - `fx_names`, `fx_params`
  - `gxa_names`, `gxa_params`
  - `gyk_names`, `gyk_params`
  - `mz_names`, `mz_params`

## 5. 処理の流れ

1. 引数が無ければ既定の `.mat` パスを使います。
2. `load(mat_path)` で係数構造体を読み込みます。
3. 出力先 `../../cpp_export` が無ければ作成します。
4. `magic_formula_params.txt` を開き、ヘッダ、入力元、日時、`Fz0`, `R0` を書きます。
5. `writeSection` を使って `Fy0`, `Fx0`, `Gxa`, `Gyk`, `Mz` を順番に書き出します。
6. ファイルを閉じて完了メッセージを表示します。

## 6. 関数・ローカル関数詳細

### `exportMagicFormulaForCpp(mat_path)`

- 全体のエクスポートを管理する公開関数です。
- 出力ファイルを開けなかった場合は `error` を送出します。

### `writeSection(fid, section_name, names, values)`

- セクション名を `[Section]` 形式で書き、その下に `name: value` を並べます。
- `names` と `values` の同じインデックスを対応付けて順に出力します。

## 7. 注意点

- 出力先ファイル名は常に `magic_formula_params.txt` 固定です。
- `.mat` のフィールド欠落に対する検証はなく、`load` 後に直接アクセスしています。
- 係数値は `%.15f` で固定小数点表記されます。

