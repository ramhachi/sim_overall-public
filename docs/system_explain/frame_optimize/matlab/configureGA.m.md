# configureGA.m
## ファイル目的
- MATLAB `ga` 用の遺伝的アルゴリズム設定を生成する。

## 実行フロー
- `pop_size`, `max_generations`, `use_parallel` の既定値を設定する。
- `optimoptions('ga', ...)` で基本設定を構築する。
- 並列が有効なら `parpool` を試行し、`UseParallel=true` を設定する。

## 関数詳細
### `options = configureGA(config)`
- シグネチャ: `function options = configureGA(config)`
- 入力:
  - `config.pop_size`
  - `config.max_generations`
  - `config.use_parallel`
- 出力:
  - `options`: `optimoptions` オブジェクト
- 責務:
  - GA 実行設定の一元化
- 主要ロジック:
  - 交叉率 0.8、エリート率 5%、標準プロット関数を設定
  - 並列プール開始失敗時は warning を出して単一スレッド継続
- 依存関係:
  - `optimoptions`, `gcp`, `parpool`
- 前提条件・注意事項:
  - 並列計算環境が無い場合も実行継続する
