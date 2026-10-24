# test_basic_functionality.m
## ファイル目的
- Week 1 基本機能の読み込み・断面計算・サイズ分類・カタログ抽出を検証する。

## 実行フロー
- 材料と管カタログをロードする。
- 円管・角管の断面計算を検証する。
- サイズ分類とカタログ絞り込みを確認する。
- サイズ分布統計を表示する。

## テスト詳細
### Test 1: `loadMaterials`
- シグネチャ: `materials = loadMaterials(...)`
- 入力: `materials.json`
- 出力: 材料 struct
- 責務: 材料読み込み確認
- 主要ロジック: `AISI4130` の `E_GPa` と密度を検証

### Test 2: `loadPipeCatalog`
- シグネチャ: `catalog = loadPipeCatalog(..., materials)`
- 入力: `pipe_catalog.json`
- 出力: 管カタログ
- 責務: 断面計算付きロード確認
- 主要ロジック: `A_mm2`, `Iy_mm4`, `size_class` の存在確認

### Test 3: 円管断面
- シグネチャ: `props = computeTubeSectionProperties(25.4, 1.65, 'round')`
- 入力: 25.4×1.65 円管
- 出力: `props`
- 責務: 円環公式確認
- 主要ロジック: 面積と断面二次モーメントを手計算と比較

### Test 4: 角管断面
- シグネチャ: `props = computeTubeSectionProperties(25.4, 1.65, 'square')`
- 入力: 25.4×1.65 角管
- 出力: `props`
- 責務: 角管公式確認

### Test 5: サイズ分類
- シグネチャ: `size_class = determineSizeClass(...)`
- 入力: 複数断面
- 出力: サイズ文字列
- 責務: A/D/None 判定確認
- 注意: 実装の Size D 条件と規則解釈差に注意しつつ、コード現状値を検証している

### Test 6: カタログ絞り込み
- シグネチャ: `idx = filterPipeCatalog(catalog, opts)`
- 入力: Size A, round, AISI4130, available
- 出力: 適合添字
- 責務: フィルタ連携確認

### Test 7: カタログ統計
- シグネチャ: ループ集計
- 入力: `catalog`
- 出力: サイズ分布
- 責務: カタログ内容 sanity check
