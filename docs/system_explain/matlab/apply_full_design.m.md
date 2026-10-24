# `matlab/apply_full_design.m`

## 1. ファイルの目的

全設計変数ベクトル `x` を、C++ シミュレータへ渡せる車両パラメータ構造体 `params` に展開する関数です。  
質量、ジオメトリ、サスペンション、空力、パワートレイン、タイヤ選択、POU、フレーム剛性、DRS をまとめて反映します。

## 2. 主な入出力または依存関係

- 入力:
  - `base_params`
  - `x`
  - `design_context`（任意）
- 出力: 更新済み `params`
- 依存関係:
  - `compute_track_width_mass_distribution`
  - `compute_aero_cd_a`
  - `apply_vehicle_extra_tunables`
  - `design_context.tire_candidates`, `tire_var_index`, `extra_tunable_specs`, `drs_var_index`

## 3. 処理の流れ / アルゴリズム

1. `base_params` をコピーし、未指定時は空の `design_context` を用意します。
2. 前半 6 変数から質量、前後重量配分、重心高、ホイールベース、前後トレッドを取り出します。
3. トレッド増加分は `compute_track_width_mass_distribution()` で sprung / unsprung に分けて質量へ反映します。
4. 慣性モーメントは総質量と車体寸法の簡易スケーリングで更新します。
5. サスペンション、ダンパ、ARB、キャンバ、トーをベクトル値から設定します。
6. `compute_aero_cd_a()` で `CL_A` に対応する `CD_A` を計算し、空力バランスを制限付きで設定します。
7. LSD、ファイナル、タイヤモデル、POU 係数、ステアリング剛性、モーションレシオ、キャンバゲイン、フレーム剛性を順次適用します。
8. `extra_tunable_specs` があれば共通拡張変数を追加適用します。
9. DRS 変数が有効なら質量・慣性増加も含めて `params.drs.enabled` を設定します。

## 4. ファイル内の関数

### `apply_full_design(base_params, x, design_context)`

- 本ファイル唯一の公開関数です。
- 先頭 32 変数程度を固定役割として解釈し、さらに `design_context.extra_tunable_specs` で可変長の共有チューニング項目を扱えます。
- サスペンション減衰 `c0` は現行車両構造が単一値前提のため、前後値の平均を採用しています。
- タイヤについては `design_context.optimize_tire_selection` が有効なときだけ候補カタログから PINN モデル情報を埋めます。
- POU 係数は前輪左右対称を仮定し、一次係数だけ左右反転させます。
- DRS はイベント依存の空力増減自体はここで扱わず、搭載重量と慣性増加のみを追加します。

## 5. 注意点・前提条件

- 設計変数の並び順がコードに直書きされているため、上流最適化器と必ず一致している必要があります。
- `base_params.steering` や `base_params.tire` が空でも補完されますが、他の下位フィールドは前提として存在する方が安全です。
- `length(x)` による条件分岐が多く、短いベクトルでは後半機能が silently にデフォルトへフォールバックします。
- DRS の空力効果は別処理前提で、この関数だけでは CL/CD の切替は行いません。
