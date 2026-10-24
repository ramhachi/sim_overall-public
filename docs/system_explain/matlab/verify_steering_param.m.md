# `matlab/verify_steering_param.m`

## 1. ファイルの目的

設計変数ベクトル中のステアリング剛性が `apply_full_design()` と `jsonencode()` を通じて正しく伝搬するか確認する検証スクリプトです。

## 2. 主な入出力または依存関係

- 入力: 固定ベクトル `x`（30 番目に 5555 を設定）
- 出力: 標準出力への struct / JSON 検証結果
- 依存関係:
  - `apply_full_design`
  - `jsonencode`

## 3. 処理の流れ / アルゴリズム

1. ステアリング剛性だけ識別しやすい値 `5555` を入れた設計変数ベクトルを作ります。
2. 最小限の `base_params` を用意して `apply_full_design()` を呼びます。
3. `params.steering.stiffness` を直接確認します。
4. さらに `jsonencode(params)` の文字列に `"stiffness":5555` が含まれるか確認します。

## 4. ファイル内の定義

### スクリプト本体

- 関数は持たず、構造体確認と JSON 文字列確認を順に行います。
- `base_params` 側に `steering` を持たせず、欠落フィールドでも反映されるかも同時に見ています。

## 5. 注意点・前提条件

- 30 番目がステアリング剛性という前提は、現在の `apply_full_design()` 実装とはずれている可能性があります。
- JSON 検索は文字列完全一致に依存する簡易検証です。
