# custom_mixed_ga.m
## ファイル目的
- 離散変数と連続変数が混在する設計ベクトル向けの自作 GA を提供する。

## 実行フロー
- 個体数・世代数・並列設定・初期推定を取得する。
- 初期集団をランダム/初期解摂動で生成し、有効個体が埋まるまで再試行する。
- 各世代で elite 保持、局所摂動、ランダム探索を組み合わせる。
- ベスト個体の `K_torsion` 履歴も取得する。

## 関数詳細
### `[best_x, best_fval, exitflag, output, population, scores] = custom_mixed_ga(fitnessfcn, nvars, lb, ub, intcon, options)`
- シグネチャ: `function [best_x, best_fval, exitflag, output, population, scores] = custom_mixed_ga(...)`
- 入力:
  - `fitnessfcn`
  - `nvars`, `lb`, `ub`
  - `intcon`: 整数変数位置
  - `options`: `PopulationSize`, `MaxGenerations`, `UseParallel`, 任意 `InitialGuess`
- 出力:
  - 最良解、最終集団、履歴付き `output`
- 責務:
  - 混合整数最適化
- 主要ロジック:
  - `elite_ratio=0.15`, `perturb_ratio=0.60`
  - 初期化は surplus 候補を大量生成して有効個体のみ採用
  - 連続変数は Gaussian 摂動、整数変数は ±1 または再乱択
- 依存関係:
  - ローカル関数 `evaluate_population`
  - `parfor`
- 前提条件・注意事項:
  - `fitnessfcn` が `Inf/NaN` を返すケースを想定
  - `history_K` は `fitnessfcn(best_x)` の第 2 出力がある前提で取得

### `scores = evaluate_population(pop, fitnessfcn, use_parallel)`
- シグネチャ: `function scores = evaluate_population(pop, fitnessfcn, use_parallel)`
- 入力:
  - `pop`, `fitnessfcn`, `use_parallel`
- 出力:
  - `scores`
- 責務:
  - 集団一括評価
- 主要ロジック:
  - `use_parallel` 時は `parfor`
- 依存関係:
  - `parfor`
- 前提条件・注意事項:
  - `fitnessfcn` はスカラー目的値を返す必要がある
