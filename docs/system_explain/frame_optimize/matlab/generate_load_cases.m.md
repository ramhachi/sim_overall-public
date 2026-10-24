# generate_load_cases.m
## ファイル目的
- FSAE フレーム評価用の代表荷重ケース群を生成する。

## 実行フロー
- 重力加速度とノード割当の既定値を設定する。
- `generate_fsae_boundary_conditions` に共通設定を渡す。
- `torsion` `lateral` `longitudinal` の 3 ケースを生成する。

## 関数詳細
### `load_cases = generate_load_cases(nodes, mass_kg, g, node_config)`
- シグネチャ: `function load_cases = generate_load_cases(nodes, mass_kg, g, node_config)`
- 入力:
  - `nodes`: ノード座標
  - `mass_kg`: 車両質量
  - `g`: 重力加速度
  - `node_config`: 前後サスペンション節点設定
- 出力:
  - `load_cases`: 名前・BC・荷重・説明を持つ struct 配列
- 責務:
  - ケース生成の薄いラッパ
- 主要ロジック:
  - トルクは `1000 Nm` 固定で `bc_config` へ追加
  - 返却順は torsion → lateral_bending → longitudinal_bending
- 依存関係:
  - `generate_fsae_boundary_conditions`
- 前提条件・注意事項:
  - `node_config` 未指定時は固定インデックス前提
