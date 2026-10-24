# `tire_fitting/preprocessor.py`

## 役割

タイヤ試験データのフィルタリング、荷重帯グループ化、IQR 外れ値除去を行う前処理モジュールです。  
ローダが読み込んだ生データを、フィッティングしやすい条件へ整えます。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `pandas.DataFrame`
  - 速度・荷重・温度・スリップ制約
- 主な出力:
  - フィルタ済み DataFrame
  - `Low` / `Medium` / `High` 荷重グループ辞書
- 例外:
  - `ValueError`: 必須列不足

## 主要クラス

### `TireDataPreprocessor`

#### `filter_data(df, min_speed=30.0, min_fz=500.0, max_sa_for_longitudinal=0.5, max_sr_for_lateral=0.01, temp_range=(40.0, 80.0))`
- 列存在に応じて順次フィルタを適用します。
- `V`, `FZ`, `SA`, `SR`, `TSTC` を見て、純縦・純横条件や温度条件を絞ります。

#### `group_by_load(df)`
- `FZ` を `Low: 600-900`, `Medium: 900-1100`, `High: 1100-1400` に分けます。
- 範囲外はどのグループにも入りません。

#### `remove_outliers_iqr(df, column, multiplier=1.5)`
- `Q1`, `Q3`, `IQR` から上下限を作り、範囲外行を除きます。

## データフロー

1. 入力 DataFrame をコピーして原データを汚さない形で処理します。
2. 条件列が存在するものだけ段階的に絞り込みます。
3. 必要に応じて荷重帯辞書や外れ値除去済み DataFrame を返します。

## システム内での位置づけ

ローダと最適化器の間に入る **品質制御層** です。  
`main.py`、`main_hybrid.py`、`validate_predictions.py` で共通利用されますが、実際の適用条件は各スクリプト側で少しずつ変えています。
