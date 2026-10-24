# `matlab/dl_tire_model/computeInputRanges.m`

## 1. 概要

`computeInputRanges` は、学習入力行列から各特徴量の最小値・最大値を求め、特徴量名と一緒に構造体で返す関数です。

## 2. ファイルの役割

- 学習データの raw 入力範囲を記録します。
- 予測時の out-of-range 検出で使うための情報をまとめます。
- 範囲をコンソールに一覧表示します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
input_ranges = computeInputRanges(X_train)
```

### 3.2 引数

- `X_train`
  - `N×5` の入力行列
  - 列順は `SR, SA, Fz, IA, TSTC` を前提としています。

### 3.3 戻り値

- `input_ranges`
  - `min`: 各列の最小値
  - `max`: 各列の最大値
  - `names`: `{'SR', 'SA', 'Fz', 'IA', 'TSTC'}`

## 4. 依存関係

- MATLAB 標準関数: `size`, `min`, `max`, `fprintf`, `error`

## 5. 処理の流れ

1. `X_train` の列数が 5 か確認します。
2. 条件を満たさない場合は `error('X_train must have 5 columns ...')` を投げます。
3. `min(X_train, [], 1)` と `max(X_train, [], 1)` で列ごとの最小値・最大値を求めます。
4. 特徴量名セル配列を設定します。
5. 5 列分のレンジをループで表示します。

## 6. 関数・ローカル関数詳細

### 6.1 `computeInputRanges`

- 列名は固定値で、入力行列側のメタ情報は見ません。
- 計算対象は正規化前データを想定していますが、コード上は数値行列ならそのまま集計します。

## 7. 注意点

- 列数しか検証していないため、列順が違っていてもエラーにはなりません。
- NaN や Inf が混ざっている場合の除外処理はこの関数内では行いません。
