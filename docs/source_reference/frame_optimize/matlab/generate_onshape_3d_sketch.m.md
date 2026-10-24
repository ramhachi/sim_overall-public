# `frame_optimize/matlab/generate_onshape_3d_sketch.m`

## ファイル目的

- Onshape 向けの **3D 中心線専用 FeatureScript snippet** を生成する。
- SolidWorks VBA マクロとは違い、**tube solid / cope cut / 端面処理は生成しない**。

## 実行フロー

1. `nodes` と `elements_geom.connectivity` を検証する
2. フレーム座標 `[Xrear+, Yright+, Zup+]` を Onshape 側の `[Xfwd+, Yup+, Zright+] = [-X, Z, Y]` へ変換する
3. 接続行列を node pair 単位で重複排除する
4. 各要素について `opPolyline(..., "points" : [p1, p2])` を 1 本ずつ出力する
5. 貼り付け手順コメント付きの `.fs` ファイルとして保存する

## 関数詳細

### `output_path = generate_onshape_3d_sketch(nodes, elements_geom, options)`
- シグネチャ: `function output_path = generate_onshape_3d_sketch(nodes, elements_geom, options)`
- 入力:
  - `nodes`: `Nx3` 節点座標
  - `elements_geom.connectivity`: `Mx2` 接続
  - `options.output_dir`, `options.filename`, `options.feature_name`, `options.coordinate_comment`
- 出力:
  - `output_path`: 生成した FeatureScript ファイルのパス
- 責務:
  - Onshape に貼り付ける 3D 中心線 feature 本体を組み立てる
- 主要ロジック:
  - `unique(sort(connectivity, 2), 'rows', 'stable')` で重複セグメントを落とす
  - `opPolyline` を要素ごとに生成する
- 注意:
  - 単体で import 可能な完全 Feature Studio ではなく、**新規 Feature Studio の header/import 行の下へ貼る snippet** を出す

### `opts = parse_options(options)`
- シグネチャ: `function opts = parse_options(options)`
- 出力: 既定値補完済み `opts`
- 責務: 出力先と feature 名の既定化

### `val = get_field(s, field_name, default_value)`
- シグネチャ: `function val = get_field(s, field_name, default_value)`
- 責務: 単純な struct field 安全参照

### `onshape_coords = frame_to_onshape(frame_coords)`
- シグネチャ: `function onshape_coords = frame_to_onshape(frame_coords)`
- 責務: CAD exporter 用の座標変換
- 主要ロジック: `[-X, Z, Y]`

### `literal = format_vector_literal(row)`
- シグネチャ: `function literal = format_vector_literal(row)`
- 責務: `vector(...)*meter` 文字列を作る

### `escaped = escape_fs_string(value)`
- シグネチャ: `function escaped = escape_fs_string(value)`
- 責務: FeatureScript 文字列リテラル向けに `"` を escape する

### `output_path = write_fs_file(lines, opts)`
- シグネチャ: `function output_path = write_fs_file(lines, opts)`
- 責務: UTF-8 の `.fs` ファイルへ保存する
- 注意:
  - `opts.output_dir` が無ければ作成する
