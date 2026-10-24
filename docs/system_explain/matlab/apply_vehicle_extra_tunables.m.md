# `matlab/apply_vehicle_extra_tunables.m`

## 1. ファイルの目的

共通の拡張車両チューニング変数群 `specs` を、設計変数ベクトル `x` から `params` へ反映する補助関数です。  
ネストフィールド更新とエンジントルク一括スケールの両方を同じ仕組みで扱います。

## 2. 主な入出力または依存関係

- 入力:
  - `params`
  - `x`
  - `specs`
- 出力: `params`
- 依存関係:
  - `set_nested_struct_field`
  - `specs(i).index`, `apply_mode`, `path`

## 3. 処理の流れ / アルゴリズム

1. `specs` が空なら何もせず返します。
2. 各 spec について `index` が有効か確認し、設計変数 `x(idx)` を取り出します。
3. `apply_mode='field'` なら `set_nested_struct_field()` で指定パスへ直接代入します。
4. `apply_mode='engine_torque_scale'` ならトルク曲線と `max_torque` を倍率更新します。
5. すべて適用後、`params.tire.relaxation.Lx_base/Ly_base` があれば `params.tire.Lx/Ly` を再同期します。

## 4. ファイル内の関数

### `apply_vehicle_extra_tunables(params, x, specs)`

- 本ファイル唯一の関数です。
- 変数位置未設定または `x` 長不足の spec は黙ってスキップします。
- `engine_torque_scale` モードでは `powertrain` 構造体の存在も補います。
- 最後の relaxation 再同期は、ベース値を変えた時に実際の `Lx/Ly` も追従させるための後処理です。

## 5. 注意点・前提条件

- `specs(i).apply_mode` が未知だと `error` で停止します。
- `engine_torque_scale` は既存トルク曲線がある場合だけ作用し、曲線未設定なら `max_torque` だけ、あるいは何も変えない可能性があります。
- `params.tire.relaxation` を使わない構造でも関数自体は動作しますが、`Lx/Ly` 自動同期は行われません。
