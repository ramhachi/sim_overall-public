# `matlab/rl_driver/applyRLEnvOverrides.m`

## 1. 役割

`applyRLEnvOverrides` は、`FSAEEnvironment` インスタンスへ外部から渡されたプロパティ上書きを安全に適用する補助関数です。  
未知のプロパティ名を弾きつつ、何を上書きしたかを人間向けログに出します。

## 2. 関数一覧

1. `applyRLEnvOverrides(env, overrides, label)`
2. `formatOverrideValue(value)`

## 3. `applyRLEnvOverrides(env, overrides, label)` 詳細

処理の流れ:

1. `label` 未指定時は `'Env override'` を使う
2. `overrides` が空なら何もしない
3. `overrides` が構造体でない場合はエラー
4. `fieldnames(overrides)` を走査
5. 各フィールドについて `isprop(env, fieldName)` で実在確認
6. 未知プロパティなら `Unknown environment override` エラー
7. 既知なら `env.(fieldName) = overrides.(fieldName)` で代入
8. `formatOverrideValue()` で値を文字列化して `fprintf` でログ出力

この関数は「何でもマージする」のではなく、環境オブジェクトが実際に持っているプロパティだけを許可します。

## 4. `formatOverrideValue(value)` 詳細

ログ出力用に override 値を文字列化するヘルパーです。

- 数値・論理でスカラなら `num2str`
- 数値・論理で配列なら `mat2str`
- 文字配列ならそのまま
- スカラ string なら `char` 化
- それ以外は `jsonencode`

これにより、単純な数値だけでなく配列や構造体も比較的読みやすく表示できます。

## 5. 実装上の要点

- 環境プロパティ名 typo を即エラーにできるため、実験設定ミスの検知に有効です。  
- 値の整合性までは検証せず、「存在するプロパティかどうか」の検証に責務を限定しています。
