# generate_fsae_boundary_conditions.m
## ファイル目的
- FSAE のねじり・横曲げ・制動・ロールオーバー相当の境界条件と荷重を生成する。

## 実行フロー
- 前後節点集合と荷重係数を `config` から取得する。
- 荷重ケース名で分岐する。
- 後輪支持条件と前輪荷重分配を構成する。

## 関数詳細
### `[bcs, loads] = generate_fsae_boundary_conditions(nodes, load_case_name, config)`
- シグネチャ: `function [bcs, loads] = generate_fsae_boundary_conditions(nodes, load_case_name, config)`
- 入力:
  - `nodes`: ノード座標
  - `load_case_name`: `torsion/lateral/longitudinal/rollover`
  - `config`: 節点割当、車重、G 値、トルク
- 出力:
  - `bcs`: `[node, dof, value]`
  - `loads`: `[node, dof, force]`
- 責務:
  - 車体試験条件のモデル化
- 主要ロジック:
  - torsion: 後輪拘束 + 前輪カップル荷重
  - lateral: 後輪拘束 + 前輪横力
  - longitudinal: 後輪拘束 + 前輪制動力
  - rollover: 下側支持 + メインフープ上部横荷重
- 依存関係:
  - なし
- 前提条件・注意事項:
  - DOF は `1=ux, 2=uy, 3=uz, 4=rx, 5=ry, 6=rz`
  - 前輪左右間隔が極小の場合、ねじりは力偶ではなく回転荷重へフォールバック
