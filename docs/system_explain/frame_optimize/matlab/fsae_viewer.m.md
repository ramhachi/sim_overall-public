# fsae_viewer.m
## ファイル目的
- フレーム 3D 描画と剛性履歴グラフを横並びに表示するダッシュボードを構成する。

## 実行フロー
- オプション引数の既定値を設定する。
- 左ペインで `plot_frame` を呼ぶ。
- BC 点と荷重点を重ね描きする。
- 右ペインで世代対剛性履歴を描き、目標線を追加する。

## 関数詳細
### `fsae_viewer(nodes, elements_geom, pipe_assignments, catalog, pareto_results, bcs, loads, stiff_targets)`
- シグネチャ: `function fsae_viewer(nodes, elements_geom, pipe_assignments, catalog, pareto_results, bcs, loads, stiff_targets)`
- 入力:
  - 幾何・接続・パイプ割当
  - `pareto_results.history_K`, `pareto_results.gen_history`
  - 任意の `bcs`, `loads`, `stiff_targets`
- 出力:
  - なし
- 責務:
  - 解析結果の複合可視化
- 主要ロジック:
  - BC は紫三角、荷重点はシアン星と矢印
  - 履歴が無ければ右ペインへメッセージのみ表示
- 依存関係:
  - `plot_frame`
  - `quiver3`, `subplot`, `yline`
- 前提条件・注意事項:
  - `pareto_results` のフィールド名は固定前提
