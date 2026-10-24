# generate_adaptive_topology.m
## ファイル目的
- 必須骨格と距離閾値ベースの追加辺から、適応的なフレーム接続グラフを生成する。

## 実行フロー
- 距離閾値と既定管仕様を初期化する。
- Front Hoop/Main Hoop/FBH/Rear Section/ブラケットなど必須辺を投入する。
- 非サスペンション節点間で距離・次数・除外領域を満たす辺を追加する。
- 次数不足節点へ救済接続を後処理で追加する。
- FEM 互換 struct へ整形する。

## 関数詳細
### `elements_geom = generate_adaptive_topology(nodes, fbh_nodes_idx, susp_nodes_idx, fh_x, mh_x, config, distance_thresholds, rear_susp_idx)`
- シグネチャ: `function elements_geom = generate_adaptive_topology(...)`
- 入力:
  - `nodes`: 全節点
  - `fbh_nodes_idx`, `susp_nodes_idx`, `rear_susp_idx`
  - `fh_x`, `mh_x`: 前後領域境界
  - `distance_thresholds`: `[R_front, R_side, R_rear]`
  - `config`: 除外領域を含む設定
- 出力:
  - `elements_geom`: `base_elements`, `connectivity`, `group_indices`, `group_names`
- 責務:
  - トポロジ候補の自動構築
- 主要ロジック:
  - 隣接行列で重複辺を防止
  - サスペンション節点は内側ブラケット以外へ直接接続させない
  - 領域ごとに group 3/4 を割り当てる
  - 各節点次数は最大 4、最低 2 を目標
- 依存関係:
  - `checkExclusionZones`
- 前提条件・注意事項:
  - 初期管仕様は固定値で、最適化後に `frameObjectiveFunction` 側で実管へ置換される
  - ノード番号の意味付けに強く依存する
