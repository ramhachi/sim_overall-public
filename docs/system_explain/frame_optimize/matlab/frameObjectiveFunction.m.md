# frameObjectiveFunction.m
## ファイル目的
- 幾何生成・トポロジ生成・材料割当・FEM・違反ペナルティを統合し、1 個体の目的関数値を評価する。

## 実行フロー
- `x` から形状・距離閾値・ノード切替・比率を分解する。
- `generate_parametric_nodes` で節点群を再生成する。
- `generate_adaptive_topology` で接続候補を構築する。
- グループごとに `catalog/materials` を割り当てて FEM 要素化する。
- 質量・重心高さ・動的荷重ケースを計算する。
- `beam_fem_solver_tcp_java` を実行し、除外領域・剛性・質量・疎性ペナルティを合算する。
- canonical flow では rear hardpoints と vehicle mass の不足を fail-fast で止める。

## 関数詳細
### `[obj_val, results] = frameObjectiveFunction(x, nodes_base, bcs, loads, config, catalog, materials, reg_params)`
- シグネチャ: `function [obj_val, results] = frameObjectiveFunction(x, nodes_base, bcs, loads, config, catalog, materials, reg_params)`
- 入力:
  - `x`: 18 変数ベクトル
  - `config`: `geom`, `suspension_file`, 剛性目標、重み、除外領域など
  - `catalog`, `materials`, `reg_params`
- 出力:
  - `obj_val`: 最小化対象の総合スコア
  - `results`: FEM 結果と各ペナルティの詳細
- 責務:
  - 評価対象個体の完全評価
- 主要ロジック:
  - 幾何生成失敗は `1e10` で棄却
  - 材料不適合は `1e7` 加算
  - `cg_z` 超過、長すぎる総延長、要素数過多にもペナルティ
  - 剛性は目標超過で報酬を与えず、目標未満のみ罰則
- 依存関係:
  - `generate_parametric_nodes`
  - `generate_adaptive_topology`
  - `checkMaterialRestriction`
  - `generate_load_cases`
  - `beam_fem_solver_tcp_java`
  - `checkExclusionZones`
- 前提条件・注意事項:
  - `nodes_base`, `bcs`, `loads` は実質未使用で互換性用
  - 質量計算は円管面積式を用いており、角管差異は未反映
  - 除外領域衝突がある個体は `Inf`
  - `config.sparsity_max_total_length_m` / `config.sparsity_max_elements` で疎性ペナルティ閾値を上書きできる
