# frame_optimize/matlab/generate_parametric_nodes.m

## 役割

設計変数からフレーム節点を具体的な 3D 座標に展開する関数です。フロントフープ、サイドインパクト、メインフープ、後部ボックス、前後サスペンション取付点、内側ブラケットを 1 つの節点配列へまとめます。

## 入力の意味

- `shape_params = [H_fh, W_si, H_si_upper, X_si_ratio]`
- `node_toggles = [Front_Mid, Rear_Mid]`
- `node_ratios = [Front_X, Front_Z, Rear_X, Rear_Z]`

ただし現在実装では **前側中間面は削除済み** で、実質的に使われる可変追加節点は後部中間面です。

## 実装の流れ

1. ホイールベースと床高さなどの固定寸法を読む
2. `load_suspension_hardpoints` で前側取付点と FBH を作る
3. `load_rear_suspension_hardpoints` で後側取付点を作る
4. そこから FH、SI、MH、後部リングの位置を計算する
5. 条件付きで rear mid nodes を追加する
6. 最後に FBH、前後サスペンション、内側ブラケット節点を末尾へ追加する

## 戻り値の意味

- `nodes`: FEM に渡す全節点
- `fbh_idx`, `susp_idx`, `rear_susp_idx`: 後段で境界条件や可視化に使うインデックス
- `X_fh`, `X_mh`: トポロジー生成の基準位置

## 実装上の注意

最適化は「節点数も変わり得る」前提なので、この関数の返すインデックス群が後続関数との接着剤になっています。
