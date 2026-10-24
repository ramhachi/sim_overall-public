# `matlab/dl_tire_model/check_symmetry.m`

## 1. 概要

`check_symmetry.m` は、Cornering と DriveBrake の実測データを読み込み、`Fy`・`Mz`・`Fx` の対称性をビン平均ベースで確認する診断用スクリプトです。

## 2. ファイルの役割

- Cornering データで `Fy(SA)` と `-Fy(-SA)` の近さを確認します。
- 同じ条件で `Mz(SA)` と `-Mz(-SA)` の近さを確認します。
- DriveBrake データで `Fx(SR)` と `-Fx(-SR)` の近さを確認します。
- あわせて SA/SR 分布も件数で表示します。

## 3. インターフェース

### 3.1 種別

スクリプトです。引数や戻り値はありません。

### 3.2 主な固定設定

- `data_dir = '../../data/tiredata/csv'`
- `tire_model = 'Hoosier_18p0X6p0-10_R20'`
- `Fz0 = -890`

### 3.3 主な内部変数

- `cr_raw`, `db_raw`: `loadTireData` で読み込んだ生データ
- `cr_pure`: `|SR| < 0.05` の Cornering データ
- `db_pure`: `|SA| < 1` の DriveBrake データ
- `cr_ref`, `db_ref`: `Fz0 ± 200` に絞った比較用データ

## 4. 依存関係

- プロジェクト関数: `loadTireData`
- MATLAB 標準関数: `dir`, `fullfile`, `sort`, `cellfun`, `contains`, `mean`, `sum`, `fprintf`

## 5. 処理の流れ

1. Cornering CSV を `Cr_<tire_model>*.csv` で列挙し、`loadTireData` で一括読込します。
2. Cornering データから `|SR| < 0.05` の行だけを `cr_pure` として残します。
3. SA 正側・負側・ゼロ近傍の件数を表示します。
4. `Fz` が `Fz0 ± 200` に入る `cr_ref` を作り、`sa_bins = [-12 -10 -8 -6 -4 -2 2 4 6 8 10 12]` で `Fy` の対称性表を出力します。
5. 同じ `cr_ref` と `sa_bins` を使って `Mz` の対称性表を出力します。
6. DriveBrake CSV を `Db_<tire_model>*.csv` で列挙し、`Run068` と `Run071` を除外してから `loadTireData` で読み込みます。
7. `|SA| < 1` の行を `db_pure` として残し、SR 分布を表示します。
8. `Fz0 ± 200` の `db_ref` を対象に、`sr_bins = [0.05 0.10 0.15 0.20]` で `Fx` の対称性表を出力します。

## 6. 関数・ローカル関数詳細

このファイルにローカル関数はありません。

### 6.1 Cornering 評価部

- `m1 = abs(cr_ref.SA - sa) < 0.5`、`m2 = abs(cr_ref.SA + sa) < 0.5` で正負の対応点を取っています。
- 正負両方にデータがあるときだけ平均値比較を表示します。

### 6.2 DriveBrake 評価部

- `Run068` と `Run071` を名前ベースで除外しています。
- `m1`, `m2` の件数がどちらも 5 を超える場合のみ対称性を表示し、不足時は「データ不足」と表示します。

## 7. 注意点

- `Fz0` は `-890` に固定されており、比較対象荷重はこの近傍だけです。
- 対称性評価は厳密な補間ではなく、±0.5 deg や ±0.015 の近傍平均による近似比較です。
- DriveBrake の除外 Run はファイル名文字列で判定しており、別命名の異常 Run には対応しません。
