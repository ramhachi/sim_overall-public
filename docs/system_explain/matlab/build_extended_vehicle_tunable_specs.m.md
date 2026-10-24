# `matlab/build_extended_vehicle_tunable_specs.m`

## 1. ファイルの目的

GA や設計適用関数で共通利用する「拡張車両チューニング変数定義一覧」を生成するファイルです。  
各変数の名前、上下限、適用方法、構造体パス、説明、単位を struct 配列として返します。

## 2. 主な入出力または依存関係

- 主入力: なし
- 主出力: `specs`
- 依存関係:
  - ローカル関数 `make_spec`
  - 下流では `apply_vehicle_extra_tunables()` が `apply_mode` と `path` を解釈します

## 3. 処理の流れ / アルゴリズム

1. `build_extended_vehicle_tunable_specs()` が `make_spec()` を並べて struct 配列を構築します。
2. ブレーキバランス、Ackermann、エンジントルク倍率、タイヤ荷重・緩和長・剛性、バンプストップ、フレーム剛性関連を順番に登録します。
3. 各 spec は初期 `index=[]` のまま返し、最適化スクリプト側で実際の変数番号を割り当てます。

## 4. ファイル内の関数

### `build_extended_vehicle_tunable_specs()`

- 公開関数です。
- 現在は 15 個の追加 spec を返し、基本設計変数だけでは表現できない車両パッケージ項目をカバーします。
- `apply_mode='field'` はネスト構造体の直接代入、`'engine_torque_scale'` は特別処理を意味します。

### `make_spec(name, lower, upper, apply_mode, path, description, unit)`

- 1 件分の spec 構造体を生成するローカルヘルパーです。
- `index` を空で初期化し、上流スクリプトが後から実際の設計変数番号を埋められるようにしています。

## 5. 注意点・前提条件

- 返却順はそのまま最適化変数並びへ使われることが多いため、追加・削除時は上流の index 割当も見直す必要があります。
- `path` は `set_nested_struct_field()` が解釈するため、ネスト階層名は実際の車両パラメータ構造体と一致していなければなりません。
