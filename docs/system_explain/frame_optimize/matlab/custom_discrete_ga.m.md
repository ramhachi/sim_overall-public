# custom_discrete_ga.m
## ファイル目的
- 離散整数設計変数専用の軽量 GA を提供する。

## 実行フロー
- ランダム整数集団を初期化する。
- 集団を評価し、elitism で上位個体を保持する。
- トーナメント選択、均一交叉、突然変異で次世代を生成する。
- 全世代の最良解を追跡する。

## 関数詳細
### `[best_x, best_fval, exitflag, output, population, scores] = custom_discrete_ga(fitnessfcn, nvars, lb, ub, options)`
- シグネチャ: `function [best_x, best_fval, exitflag, output, population, scores] = custom_discrete_ga(...)`
- 入力:
  - `fitnessfcn`
  - `nvars`, `lb`, `ub`
  - `options.PopulationSize`, `options.MaxGenerations`, `options.UseParallel`
- 出力:
  - 最良解・評価値・最終集団
- 責務:
  - 離散カタログ選択最適化
- 主要ロジック:
  - 突然変異率 15%
  - エリート率 5%
  - 各遺伝子は突然変異時に範囲内乱数へ置換
- 依存関係:
  - `evaluate_population`
  - `tournament_select`
- 前提条件・注意事項:
  - 連続変数には非対応

### `scores = evaluate_population(pop, fitnessfcn, use_parallel)`
- シグネチャ: `function scores = evaluate_population(pop, fitnessfcn, use_parallel)`
- 入力:
  - `pop`, `fitnessfcn`, `use_parallel`
- 出力: `scores`
- 責務:
  - 集団評価
- 主要ロジック:
  - 並列可否で `parfor`/`for` を切替
- 依存関係:
  - `parfor`
- 前提条件・注意事項:
  - 目的関数が例外を出すと全体が停止する

### `parent = tournament_select(pop, scores, k)`
- シグネチャ: `function parent = tournament_select(pop, scores, k)`
- 入力:
  - `pop`, `scores`, `k`
- 出力:
  - `parent`
- 責務:
  - 親個体選択
- 主要ロジック:
  - `randperm` で `k` 個を抽出し、最良スコア個体を返す
- 依存関係:
  - `randperm`
- 前提条件・注意事項:
  - `k <= population size`
