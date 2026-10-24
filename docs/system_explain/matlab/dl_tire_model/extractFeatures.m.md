# `matlab/dl_tire_model/extractFeatures.m`

## 1. 概要

タイヤ試験データ表から、学習入力 `SR, SA, Fz, IA, TSTC` を所定順で抜き出す関数です。  
入力型と必要列名の検証を含みます。

## 2. ファイルの役割

- 前処理の入口で、テーブル形式データを数値行列へ変換します。
- 特徴量列の並び順を固定し、後段処理が期待する `N×5` 行列を作ります。

## 3. インターフェース

### 関数 `X = extractFeatures(data)`

- 入力
  - `data`: `SR`, `SA`, `Fz`, `IA`, `TSTC` を含む table
- 出力
  - `X`: `N×5` の特徴量行列 `[SR, SA, Fz, IA, TSTC]`

## 4. 依存関係

- MATLAB 標準機能: `istable`, `class`, `setdiff`, `strjoin`
- 外部独自関数には依存しません。

## 5. 処理の流れ

1. 入力が table かどうかを `istable` で確認します。
2. 必須列名 `{'SR','SA','Fz','IA','TSTC'}` を定義します。
3. `data.Properties.VariableNames` と比較し、欠落列があれば `error` を送出します。
4. 必須 5 列をこの順序で連結し、`X` を返します。

## 6. 関数・ローカル関数詳細

### `extractFeatures(data)`

- ファイル内唯一の関数です。
- 列存在確認と抽出順の固定を同時に担います。
- エラー ID は `extractFeatures:InvalidInput` と `extractFeatures:MissingColumns` です。

## 7. 注意点

- 列名が一致していても単位系や値域の妥当性までは検証しません。
- 出力順はテーブル側の列順ではなく、コードで固定された順番です。

