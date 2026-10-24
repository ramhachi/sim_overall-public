# `matlab/extract_vehicle_params_from_artifact.m`

## 1. ファイルの目的

artifact 構造体と plain params 構造体の両方を受け取り、最終的に車両パラメータ構造体だけを取り出す正規化関数です。

## 2. 主な入出力または依存関係

- 入力: `data`
- 出力: `params`
- 依存関係: 外部関数なし

## 3. 処理の流れ / アルゴリズム

1. 入力が struct か確認し、そうでなければ即エラーにします。
2. `data.params` が struct なら artifact と見なして `data.params` を返します。
3. それ以外は `data` 自体を params とみなして返します。

## 4. ファイル内の関数

### `extract_vehicle_params_from_artifact(data)`

- 本ファイル唯一の関数です。
- artifact / plain params の二形態を吸収するだけに役割を絞っています。

## 5. 注意点・前提条件

- `params` フィールドがあっても struct でなければ artifact 扱いしません。
- JSON デコード結果が struct であることを前提にしており、配列や文字列は受け付けません。
