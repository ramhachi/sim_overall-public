# `matlab/apply_tradeoff_design.m`

## 1. ファイルの目的

`ConceptTradeOffs()` で定義した離散トレードオフと連続チューニング変数を、車両パラメータ構造体へ反映する関数です。  
共有コンセプト変数とイベント別セットアップ変数を 1 本のベクトルから展開します。

## 2. 主な入出力または依存関係

- 入力:
  - `base_params`
  - `x`
  - `config`
  - `design_context`（任意）
- 出力: `params`
- 依存関係:
  - `compute_track_width_mass_distribution`
  - `compute_aero_cd_a`
  - `apply_vehicle_extra_tunables`
  - `config.aero_packages`, `config.engine_map`

## 3. 処理の流れ / アルゴリズム

1. 共通拡張チューニング変数数を `design_context.shared_extra_specs` から数え、後段のオフセットを決めます。
2. 8 番目の整数変数を空力パッケージ ID として丸め、範囲内へクリップします。
3. 共有変数から質量、重量配分、重心高、ホイールベース、トレッド、ファイナルを取得します。
4. `compute_track_width_mass_distribution()` でトレッド起因の sprung / unsprung 増分を計算します。
5. サスペンション、アライメント、LSD、ファイナル、エンジントルクマップをイベント別チューニング値から設定します。
6. 空力パッケージの能力上限内で `CL_A`, `CD_A`, `aero_balance` を決めます。
7. 共通追加変数があれば `apply_vehicle_extra_tunables()` で適用し、最後に tire μ のデフォルトを補完します。

## 4. ファイル内の関数

### `apply_tradeoff_design(base_params, x, config, design_context)`

- 本ファイル唯一の関数です。
- 共有部とイベント別セットアップ部を単一ベクトルで扱うため、`tunable_offset = 8 + shared_tunable_count` を使って後半変数の位置をずらします。
- 空力パッケージは離散変数で選びますが、ダウンフォース量自体は `aero_frac` で連続的に調整します。
- `params.powertrain.engine_rpm_points` と `engine_torque_curve` は `config.engine_map` があれば上書きされます。
- `params.tire.mu_x_max` / `mu_y_max` は、未設定時だけ既定値 1.6 / 2.0 に補完されます。

## 5. 注意点・前提条件

- `x` の後半 15 変数はスプリング、ダンパ、ARB、キャンバ、トー、LSD、空力設定の順で固定解釈されます。
- 空力 ID は整数として扱いますが、GA などが実数を返しても四捨五入で吸収する設計です。
- 共有追加変数数が変わると後段の全インデックスがずれるため、上流最適化器と同時管理が必要です。
