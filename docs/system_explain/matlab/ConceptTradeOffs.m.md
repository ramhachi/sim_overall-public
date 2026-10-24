# `matlab/ConceptTradeOffs.m`

## 1. ファイルの目的

車両コンセプト最適化で使うトレードオフ設定を 1 つの構造体 `config` にまとめて返す関数です。  
空力パッケージの離散候補、トレッド拡大に伴う質量ペナルティ、重心高感度、エンジントルクマップを MATLAB 側の共通設定として定義します。

## 2. 主な入出力または依存関係

- 入力: なし
- 出力: `config`
  - `aero_packages`
  - `track_width_sprung_penalty`
  - `track_width_unsprung_penalty`
  - `track_width_penalty`, `unsprung_penalty`（互換用）
  - `mu_gain_per_meter`
  - `hcg_mass_sensitivity`
  - `engine_map.rpm`, `engine_map.torque`
- 依存関係: 外部関数呼び出しなし。`apply_tradeoff_design.m` や `optimize_multi_event.m` から参照される前提です。

## 3. 処理の流れ / アルゴリズム

1. 空の `config` 構造体を作成します。
2. `aero_packages` に None / Light / Heavy の 3 離散候補を struct 配列で定義します。
3. トレッド拡大量に応じた sprung / unsprung の質量増加係数を設定します。
4. 互換性維持のため旧フィールド名にも同じ値を入れます。
5. 実効 μ 補正用係数と重心高感度を設定します。
6. 最後にエンジン回転数配列とトルク配列を `engine_map` に格納して返します。

## 4. ファイル内の定義

### `ConceptTradeOffs()`

- 本ファイル唯一の公開関数です。
- 最適化スクリプトが直接編集しやすいよう、設定値をハードコードした構造体として返します。
- `aero_packages` の各要素には `name`, `mass_penalty`, `max_cl_a`, `cd_a_base`, `balance_min`, `balance_max` が入り、空力選択と空力バランス制約の両方を表現します。
- `track_width_sprung_penalty` と `track_width_unsprung_penalty` は `compute_track_width_mass_distribution()` と組み合わせて、トレッド拡幅を単なる grip 向上ではなく重量増と結びつけるために使われます。
- `engine_map` は `apply_tradeoff_design()` 側で `params.powertrain.engine_rpm_points` / `engine_torque_curve` に流し込まれます。

## 5. 注意点・前提条件

- 空力候補数は `apply_tradeoff_design()` 側で整数 ID として参照されるため、並び順が意味を持ちます。
- `track_width_penalty` と `unsprung_penalty` は旧スクリプト互換用であり、新規コードは `track_width_sprung_penalty` / `track_width_unsprung_penalty` を参照します。
- `engine_map` はファイル読込ではなく固定配列なので、実測 CSV を反映したい場合はこの関数の値自体を更新する必要があります。
