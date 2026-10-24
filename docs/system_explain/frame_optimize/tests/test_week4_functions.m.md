# test_week4_functions.m
## ファイル目的
- Week 4 の可視化関数群が非表示ウィンドウ環境でも落ちずに実行できるか確認する。

## 実行フロー
- ダミーの 5 ノードピラミッドと簡易パレートデータを作る。
- `plot_pareto`, `plot_frame`, `fsae_viewer` を順に実行し、例外有無で判定する。

## テスト詳細
### スクリプト初期化
- シグネチャ: スクリプト
- 入力: データパス、MATLAB パス
- 出力: `nodes`, `elems_geom`, `pipe_assign`, `p_results`
- 責務: 可視化テストの準備

### Test 1: `plot_pareto`
- シグネチャ: `plot_pareto(p_results.history, p_results.front, 'Unit Test Pareto')`
- 入力: ダミー履歴/前線
- 出力: 非表示 Figure
- 責務: パレート描画のクラッシュ確認

### Test 2: `plot_frame`
- シグネチャ: `plot_frame(nodes, elems_geom, pipe_assign, catalog, true)`
- 入力: ピラミッド形状
- 出力: 非表示 Figure
- 責務: 3D 円柱描画のクラッシュ確認

### Test 3: `fsae_viewer`
- シグネチャ: `fsae_viewer(nodes, elems_geom, pipe_assign, catalog, p_results)`
- 入力: ダミー可視化データ
- 出力: ダッシュボード Figure
- 責務: 統合ビューアの実行確認
- 注意: `DefaultFigureVisible` を一時変更する
