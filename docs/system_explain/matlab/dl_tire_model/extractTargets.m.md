# `matlab/dl_tire_model/extractTargets.m`

## 1. 概要

タイヤ試験データ表から、教師信号 `Fx, Fy, Mx, Mz` を取り出す関数です。  
`extractFeatures` と同様に、型と列存在を検証してから行列化します。

## 2. ファイルの役割

- 学習ターゲットをテーブルから `N×4` 数値行列へ変換します。
- 出力列の並び順を固定し、学習・評価処理の入力形式を揃えます。

## 3. インターフェース

### 関数 `Y = extractTargets(data)`

- 入力
  - `data`: `Fx`, `Fy`, `Mx`, `Mz` を含む table
- 出力
  - `Y`: `N×4` の教師行列 `[Fx, Fy, Mx, Mz]`

## 4. 依存関係

- MATLAB 標準機能: `istable`, `class`, `setdiff`, `strjoin`
- 外部独自関数には依存しません。

## 5. 処理の流れ

1. 入力が table かどうかを確認します。
2. 必須列名 `{'Fx','Fy','Mx','Mz'}` を定義します。
3. 欠落列があれば `extractTargets:MissingColumns` の `error` を送出します。
4. 4 列を `[data.Fx, data.Fy, data.Mx, data.Mz]` の順で結合して返します。

## 6. 関数・ローカル関数詳細

### `extractTargets(data)`

- ファイル内唯一の関数です。
- エラー ID は `extractTargets:InvalidInput` と `extractTargets:MissingColumns` を使います。
- 出力順が固定されているため、後段の評価コードは列位置前提で扱えます。

## 7. 注意点

- 力・モーメントの単位や符号系はこの関数では検証しません。
- 必要列があれば追加列が存在しても無視されます。

