# plot_pareto.m
## ファイル目的
- 全評価点とパレート前線を同一図へ描画し、質量とねじり剛性のトレードオフを可視化する。

## 実行フロー
- タイトル既定値を設定する。
- 全サンプルを灰色散布図で描画する。
- パレート点を質量順にソートし、折れ線と強調点で描画する。
- タイトル・軸ラベル・凡例を設定する。

## 関数詳細
### `plot_pareto(history_results, pareto_front, title_str)`
- シグネチャ: `function plot_pareto(history_results, pareto_front, title_str)`
- 入力:
  - `history_results`: `[mass, stiffness]` の評価履歴
  - `pareto_front`: パレート前線点
  - `title_str`: 任意タイトル
- 出力:
  - なし
- 責務:
  - 最適化履歴の可視化
- 主要ロジック:
  - 背景散布図と前線を分離表示
  - 前線は `sortrows(...,1)` で質量昇順
- 依存関係:
  - `scatter`, `plot`, `legend`
- 前提条件・注意事項:
  - 1 列目が質量、2 列目が剛性である前提
