# test_week2_functions.m
## ファイル目的
- Week 2 実装の規則関連関数と基本目的関数を検証する。

## 実行フロー
- 材料・管カタログ・規則 JSON をロードする。
- サイズ要求、材料制約、除外領域、目的関数を順にテストする。

## テスト詳細
### Test 1: `getRequiredSizeClass`
- シグネチャ: `size_mh = getRequiredSizeClass('main_hoop', reg_params)` など
- 入力: `reg_params`
- 出力: サイズ区分
- 責務: 規則マップ読取り確認
- 主要ロジック: `main_hoop=A`, `front_bulkhead=B`, `diagonal_brace=C`, 不明=`None`

### Test 2: `checkMaterialRestriction`
- シグネチャ: `[is_valid, reason] = checkMaterialRestriction(...)`
- 入力: 鋼パイプ、厚肉 Al、薄肉 Al
- 出力: 適合可否
- 責務: 材料規則確認
- 主要ロジック:
  - main hoop + steel は pass
  - main hoop + aluminum は fail
  - front bulkhead + 厚肉 Al は pass
  - front bulkhead + 薄肉 Al は fail

### Test 3: `checkExclusionZones`
- シグネチャ: `penalties = checkExclusionZones(nodes, elements, config)`
- 入力: 3 ノード 2 要素、AABB 1 個
- 出力: ペナルティ
- 責務: 交差長計算確認
- 主要ロジック: 1 本目のみ 200 mm 相当の侵入罰則を確認

### Test 4: `frameObjectiveFunction`
- シグネチャ: `[obj_val, results] = frameObjectiveFunction(...)`
- 入力: 極小ダミーフレーム
- 出力: 目的値と結果
- 責務: 基本評価のスモーク確認
- 主要ロジック: `results.success` と極端罰則がないことを確認
- 注意: 現在の `frameObjectiveFunction` 実装は本テストのダミー入力前提と完全一致しないため、テストはコード変更に影響されやすい
