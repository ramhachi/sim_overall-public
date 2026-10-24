# checkExclusionZones.m
## ファイル目的
- 線分要素が AABB/Cylinder 型の除外領域へ侵入する長さを計算し、ペナルティへ変換する。

## 実行フロー
- 出力配列を初期化する。
- 除外領域定義を正規化する。
- 各要素・各ゾーンについて交差長を計算する。
- 交差長 × `1e6` を要素ペナルティとして返す。

## 関数詳細
### `[penalties, collision_names] = checkExclusionZones(nodes, elements, config, group_indices)`
- シグネチャ: `function [penalties, collision_names] = checkExclusionZones(nodes, elements, config, group_indices)`
- 入力:
  - `nodes`, `elements`
  - `config.exclusion_zones`
  - `group_indices`: 要素グループ ID（任意）
- 出力:
  - `penalties`: 要素ごとの侵入ペナルティ
  - `collision_names`: 最初に衝突したゾーン名
- 責務:
  - ジオメトリ干渉判定
- 主要ロジック:
  - AABB は slab 法、Cylinder は XY 投影の二次方程式で判定
  - `front_hoop_sweep` と `driver_cockpit` にはグループ免除がある
- 依存関係:
  - `warning`, `regexp`
- 前提条件・注意事項:
  - 要素長 0 は無視
  - 円柱の Z 重なり判定は簡略化されている
