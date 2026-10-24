# test_generate_solidworks_macro.m
## ファイル目的
- `generate_solidworks_macro` が実データと合成フレームを用いて `.swb` を正しく生成できるか総合検証する。

## 実行フロー
- 材料・管カタログをロードする。
- 59 ノードの合成フレームと `elements_geom` を手動構築する。
- `best_x` と出力先 options を設定する。
- 11 個の検証セクションで生成 VBA の構造と内容を確認する。
- 最後に一時出力ディレクトリを削除する。

## テスト詳細
### `test_generate_solidworks_macro()`
- シグネチャ: `function test_generate_solidworks_macro()`
- 入力: なし
- 出力: なし
- 責務: SolidWorks マクロ生成の包括テスト
- 依存関係: `loadMaterials`, `loadPipeCatalog`, `generate_solidworks_macro`
- 注意: `tempdir` 配下を使用しており、純粋なリポジトリ内検証ではない

### Setup セクション
- 入力: `materials.json`, `pipe_catalog.json`
- 出力: `nodes`, `elements_geom`, `best_x`, `opts`
- 責務: 実運用に近い入力データの用意
- 主要ロジック: Front/Main Hoop, FBH, 前後サス点を固定ノード番号で構築

### Test 1: 関数実行
- シグネチャ: `output_path = generate_solidworks_macro(...)`
- 期待値: 例外なく `.swb` パスが返る
- 責務: 呼び出し可能性検証

### Test 2: 出力ファイル存在とサイズ
- シグネチャ: `assert(isfile(output_path))`
- 期待値: `.swb` が作成され 1000 byte 超
- 責務: ファイル生成検証

### Test 3: VBA 構造検証
- 対象:
  - `.swb` 形式で `Attribute VB_Name` が無い
  - `Sub/End Sub` と `Function/End Function` が平衡
  - loop 内 `Dim` が無い
- 責務: コンパイル前静的検査

### Test 4: ノードデータ検証
- 対象:
  - `NUM_NODES`
  - Node 1 の座標変換 `[-X, Z, Y]`
- 責務: 座標系変換確認

### Test 5: 必須 VBA セクション存在
- 対象:
  - `InitNodes`, `CreateCustomProfiles`, `Build_*`, `TrimMembers`, `Main`
- 責務: マクロ生成 completeness 検証

### Test 6: プロファイル定数
- 対象: `PIPE_OD_1`, `PIPE_T_*`
- 責務: カタログ連携確認

### Test 7: フィレット生成
- 対象: `CreateFillet` 呼び出し数 7 以上
- 責務: bent hoop の曲げ部生成確認

### Test 8: `InsertStructuralWeldment5`
- 対象: 呼び出し数 10 以上
- 責務: 各部材群が weldment 化されることの確認

### Test 9: パス文字列
- 対象: `\\` が含まれない
- 責務: VBA 文字列の Windows パス表現検証

### Test 10: `Main` 実行順序
- 対象: `InitNodes` → `CreateCustomProfiles` → 各 `Build_*` → `TrimMembers` → `ExportBodies`
- 責務: マクロ組立順序保証

### Test 11: late binding
- 対象:
  - `SldWorks.xxx` 型参照なし
  - `swXxx_e.` 参照なし
  - 代替定数定義あり
- 責務: `.swb` 互換性確認
