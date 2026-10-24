# `matlab/set_nested_struct_field.m`

## 1. ファイルの目的

ドット区切りパスまたはセル配列パスを使って、ネストした struct フィールドへ動的に値を書き込むユーティリティです。

## 2. 主な入出力または依存関係

- 入力: `s`, `field_path`, `value`
- 出力: `s`
- 依存関係: `subsasgn`, `substruct`

## 3. 処理の流れ / アルゴリズム

1. `field_path` が文字列なら `strsplit('.', ...)` で分解します。
2. 各階層名を `substruct` 用の `'.', fieldname` ペア列へ変換します。
3. `subsasgn()` で対象フィールドへ `value` を代入した struct を返します。

## 4. ファイル内の関数

### `set_nested_struct_field(s, field_path, value)`

- 本ファイル唯一の関数です。
- `{'tire','relaxation','Lx_base'}` のようなセルパスと、`'tire.relaxation.Lx_base'` のような文字列パスの両方を受け付けます。

## 5. 注意点・前提条件

- 中間階層が存在しない場合の `subsasgn` 挙動に依存します。
- 配列インデックスではなくフィールド階層専用です。
