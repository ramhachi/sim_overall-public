# loadPipeCatalog.m
## ファイル目的
- `pipe_catalog.json` を読み込み、各管材へ断面特性・サイズ区分を付与したカタログ配列を返す。

## 実行フロー
- 入力数と型を検証する。
- JSON を読み込み、`pipes` フィールドの存在を確認する。
- 各 pipe について必須フィールドを検証する。
- `computeTubeSectionProperties` で `A/I/J` を計算する。
- `determineSizeClass` で FSAE サイズ区分を付与する。
- 既知材料かを確認し、未知材は警告する。

## 関数詳細
### `catalog = loadPipeCatalog(json_path, materials)`
- シグネチャ: `function catalog = loadPipeCatalog(json_path, materials)`
- 入力:
  - `json_path`: カタログ JSON パス
  - `materials`: `loadMaterials` が返す材料 struct
- 出力:
  - `catalog`: 各管材の基本情報・計算断面・`size_class` を持つ struct 配列
- 責務:
  - JSON 取り込み
  - 必須項目検証
  - 断面計算と分類
- 主要ロジック:
  - cell 配列/struct 配列の両形式に対応
  - 任意項目 `cost_per_meter_jpy` `lead_time_days` は存在時のみ転記
  - 材料 ID は `matlab.lang.makeValidName` で field 名へ変換
- 依存関係:
  - `computeTubeSectionProperties`
  - `determineSizeClass`
  - `jsondecode`, `fileread`
- 前提条件・注意事項:
  - JSON には `pipes` フィールドが必要
  - `materials` に未知の材料があっても処理は継続し警告のみ
