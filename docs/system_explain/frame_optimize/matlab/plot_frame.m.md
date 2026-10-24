# plot_frame.m
## ファイル目的
- フレーム要素を実外径付き円柱として 3D 描画する。

## 実行フロー
- ノード表示有無と色マップを設定する。
- 各要素について使用パイプを決め、円柱基底メッシュを要素方向へ回転変換する。
- サイズ区分色で `surf` 描画する。
- 必要ならノード散布図を追加する。

## 関数詳細
### `plot_frame(nodes, elements_geom, pipe_assignments, catalog, show_nodes)`
- シグネチャ: `function plot_frame(nodes, elements_geom, pipe_assignments, catalog, show_nodes)`
- 入力:
  - `nodes`
  - `elements_geom.connectivity`, `group_indices`, 任意で `base_elements`
  - `pipe_assignments`
  - `catalog`
  - `show_nodes`
- 出力:
  - なし
- 責務:
  - 最適化結果の立体可視化
- 主要ロジック:
  - `group_id==0` は固定要素として `base_elements` から寸法取得
  - 円柱初期軸 `[0 0 1]` を Rodrigues 型回転で要素方向へ合わせる
  - 色は Size A/B/C/D/None で固定
- 依存関係:
  - `cylinder`, `surf`, `scatter3`
- 前提条件・注意事項:
  - 描画形状は常に円柱で、角管断面形状は反映しない
