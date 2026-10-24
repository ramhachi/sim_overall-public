# `matlab/dl_tire_model/validateTireData.m`

## 1. 概要

`validateTireData` は、タイヤ試験データの必須列、有効値数、欠損行数、各列の統計レンジを確認する関数です。結果は `stats` 構造体にまとめられ、要約が標準出力へ表示されます。

## 2. ファイルの役割

前処理や学習前に、入力テーブルが必要列を持ち、`NaN` / `Inf` の状況が把握できるかを確認するバリデータです。どの列に欠損がどれだけあるか、全体で何行が汚染されているかを集計します。

## 3. インターフェース

### 関数シグネチャ

```matlab
stats = validateTireData(data)
```

### 引数

- `data`
  - タイヤ試験データを持つ table。

### 戻り値 `stats`

- `total_samples`
  - 総行数。
- `missing_by_column`
  - 必須各列ごとの欠損/無限大件数。
- `rows_with_missing`
  - どれか 1 列でも `NaN` または `Inf` を含む行数。
- `clean_samples`
  - 欠損のない行数。
- `ranges`
  - 各列の `min`, `max`, `mean`, `std`。

### エラー条件

- `isempty(data)`
  - `validateTireData:EmptyData`
- 必須列不足
  - `validateTireData:MissingColumns`

## 4. 依存関係

- MATLAB 組み込み
  - `isempty`, `error`, `setdiff`, `height`, `isnan`, `isinf`, `table2array`, `any`, `min`, `max`, `mean`, `std`, `warning`, `fprintf`

## 5. 処理の流れ

1. `stats` を空構造体で初期化します。
2. `data` が空ならエラーにします。
3. 必須列 `{'SR','SA','Fz','IA','TSTC','Fx','Fy','Mx','Mz'}` を定義します。
4. `setdiff` で不足列を調べ、不足があればエラーにします。
5. 総サンプル数 `stats.total_samples` を記録します。
6. 各必須列について `NaN` 件数と `Inf` 件数を数え、`missing_by_column` に格納します。
7. 欠損がある列は `warning` で列名と割合を通知します。
8. `table2array(data)` から全行を数値配列化し、どこかに `NaN` / `Inf` を含む行を `bad_rows` として数えます。
9. 各列の有効値だけを使って `min`, `max`, `mean`, `std` を `stats.ranges` に格納します。
10. 最後にサマリとレンジ一覧を `fprintf` で表示します。

## 6. 関数・ローカル関数詳細

### `validateTireData`

- 必須列の存在確認と、値の健全性確認を 1 回で行う関数です。
- 列ごとの欠損件数だけでなく、行単位の汚染件数も返すため、後続で除外率を判断しやすくなっています。
- ローカル関数はありません。

## 7. 注意点

- `table2array(data)` を使うため、テーブル全体が数値化可能であることを前提にしています。
- `ranges` は必須列のみを対象とし、それ以外の補助列は評価しません。
- 欠損が存在しても、この関数自体は警告を出すだけで自動除去は行いません。
