# `frame_optimize/examples/diagnose_optimization.m`

## 役割

- `frameObjectiveFunction` が見ている節点・荷重・トポロジーが意図通りかを、最適化前に診断するデバッグ用スクリプトです。

## スクリプトの流れ

- 実車ジオメトリ、材料、カタログ、懸架ファイルを読み込みます。
- 基準の `shape_params`、`node_toggles`、`node_ratios` で `generate_parametric_nodes` を実行します。
- 節点数、FBH/FH/MH、前後懸架マウントの節点番号と座標を表示します。
- `generate_load_cases` に渡す `node_config` を作り、拘束・荷重点を確認します。
- `generate_adaptive_topology` でトポロジーを生成し、拘束点と荷重点が接続されているかを確認します。
- 最後に FEM サーバを起動し、実際に 1 回解いて剛性と質量を表示します。

## 入力

- `real_vehicle_geometry.json`
- `materials.json`
- `pipe_catalog.json`
- `suspension_base_geometry.json`
- 手書き基準パラメータ

## 出力

- 節点番号と座標のログ
- 荷重ケースの拘束点・荷重点一覧
- トポロジー接続確認
- 1 回分の FEM 結果

## セクション詳細

### ノード生成確認

- **責務**
  - 最適化対象フレームで、前後懸架マウントや内部ブラケットの番号ずれが起きていないか確認します。

### 荷重ケース確認

- **責務**
  - `generate_load_cases` に渡す `node_config` が、前後左右の対応を正しく持っているかを検査します。

### 接続性確認

- **責務**
  - 拘束点や荷重点が `elements_geom.connectivity` に本当に含まれているかを `ismember` で確認します。
- **重要性**
  - 接続されていない節点に荷重を載せると、FEM 結果が不安定または無意味になります。

### FEM 試験実行

- **責務**
  - 最低限の設定で `beam_fem_solver_tcp_java` が計算まで通るかを確認します。

## 前提・注意点

- 本番最適化ではなく診断専用です。
- サーバ起動・停止を行うため、並行実行中の他スクリプトと衝突する可能性があります。
