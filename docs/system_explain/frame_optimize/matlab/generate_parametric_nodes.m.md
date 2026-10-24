# generate_parametric_nodes.m
## ファイル目的
- 車両寸法・最適化形状変数・サスペンション hardpoint から可変ノード集合を生成する。

## 実行フロー
- 車両寸法、FBH 寸法、最適化パラメータを展開する。
- 前後サスペンション hardpoint を読み込む。
- Front Hoop, Side Impact, Main Hoop, Rear Section の基準節点を組む。
- rear mid-plane を条件付きで追加する。
- FBH、前後サスペンション点、内側ブラケットを追記し、インデックスを返す。

## 関数詳細
### `[nodes, fbh_nodes, suspension_mount_nodes, inner_bracket_nodes, X_fh, X_mh, fbh_idx, susp_idx, rear_susp_idx] = generate_parametric_nodes(geom, suspension_file, shape_params, node_toggles, node_ratios)`
- シグネチャ: `function [nodes, ...] = generate_parametric_nodes(...)`
- 入力:
  - `geom`: `geom.dimensions` を含む車両形状
  - `suspension_file`: サスペンション JSON
  - `shape_params`: `[H_fh, W_si, H_si_upper, X_si_ratio]`
  - `node_toggles`: `[Has_Front_Mid, Has_Rear_Mid]`
  - `node_ratios`: mid-plane 配置比
- 出力:
  - `nodes`: 全節点
  - `fbh_nodes`, `suspension_mount_nodes`, `inner_bracket_nodes`
  - `X_fh`, `X_mh`, `fbh_idx`, `susp_idx`, `rear_susp_idx`
- 責務:
  - 幾何パラメータからフレーム節点を一意生成
- 主要ロジック:
  - 前輪側ではアーム方向へ 50 mm 延長して inner bracket を作る
  - `X_mh = X_fh + 0.60`
  - 後部中間リングは rear gap が 0.3 m 超時のみ追加
- 依存関係:
  - `load_suspension_hardpoints`
  - `load_rear_suspension_hardpoints`
- 前提条件・注意事項:
  - 前輪サスペンションデータが無い場合は `error`
  - ノード番号意味付けが後続処理の前提になっている
