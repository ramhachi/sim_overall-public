# generateGroundStructure.m
## ファイル目的
- 全節点対から長さ制約と除外領域制約を満たす ground structure を生成する。

## 実行フロー
- 全ノード対を列挙する。
- `max_length_mm` 以下かつ 0 長でない辺のみ採用する。
- 必要なら `checkExclusionZones` で除外領域交差辺を除去する。

## 関数詳細
### `elements = generateGroundStructure(nodes, config)`
- シグネチャ: `function elements = generateGroundStructure(nodes, config)`
- 入力:
  - `nodes`: `Nx3`
  - `config.max_length_mm`
  - `config.exclusion_zones`
- 出力:
  - `elements`: `Mx2` 接続表
- 責務:
  - 位相最適化の候補辺集合生成
- 主要ロジック:
  - `n(n-1)/2` を上限に事前確保
  - 除外領域交差辺は完全除去
- 依存関係:
  - `checkExclusionZones`
- 前提条件・注意事項:
  - 関数名やコメントは mm 前提だが、入力座標の単位整合に注意が必要
