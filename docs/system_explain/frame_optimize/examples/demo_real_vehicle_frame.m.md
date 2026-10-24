# `frame_optimize/examples/demo_real_vehicle_frame.m`

## 役割

- Formula TMU の実車寸法と懸架情報を使ってフレーム節点群を生成し、実車ベースの最適化問題を準備する大型デモです。
- ノード生成、阻止領域検証、最適化設定、FEM サーバ起動、GA 実行、ビューア表示までを含みます。

## スクリプト全体の流れ

- **1. 実車ジオメトリと材料の読込**
  - `real_vehicle_geometry.json`、材料、パイプカタログをロードします。
  - 地上高分だけ除外ゾーンを上方へシフトします。
- **2. 実車フレームノード生成**
  - FBH、FH、SI、MH、リアブレース、懸架マウント、インナーブラケットを動的に配置します。
  - フロントフープ位置は前懸架マウントの最大 X を使って後方へ逃がします。
- **3. 除外ゾーン確認**
  - 既存除外ゾーンに加え、T.1.2.1 の内部断面パスを追加して節点侵害を確認します。
- **4. 荷重ケース**
  - コメント上は FSAE 正しい BC/荷重系を扱う想定ですが、この版では動的生成へ移行したため静的配列は空で保持します。
- **5. 最適化設定**
  - 目的関数重み、剛性閾値、GA 設定、除外ゾーン、CG 上限制約を設定します。
- **6. 混合整数最適化**
  - FEM サーバを起動し、18 変数の `custom_mixed_ga` を実行します。
- **7. 最終解可視化**
  - 最適設計から節点・トポロジー・境界条件を再生成し、`fsae_viewer` へ渡します。
- **8. FEM サーバ停止**

## 主な入力

- `real_vehicle_geometry.json`
- `materials.json`
- `pipe_catalog.json`
- `suspension_base_geometry.json`
- `generate_parametric_nodes`
- `generate_adaptive_topology`
- `frameObjectiveFunction`
- `custom_mixed_ga`

## 主な出力

- `best_x`
- `best_fval`
- `output.history`, `output.history_K`
- `fsae_optimized_dashboard.png`
- `fsae_viewer` によるダッシュボード

## セクション詳細

### 2. 実車フレームノード生成

- **出力**
  - `nodes`
  - `fbh_nodes_idx`
  - `susp_nodes_idx`
- **責務**
  - 前懸架と干渉しない FBH/FH 配置、コックピット後端に置く MH、リア補剛部を一貫した実車座標系で構築します。
- **注意点**
  - 全要素は鋼で評価する前提です。
  - 要素接続はここでは固定せず、後段 `generate_adaptive_topology` に委ねています。

### 3. 除外ゾーン確認

- **責務**
  - 節点が既存除外ゾーンや T.1.2.1 の経路領域に入っていないかを検証します。
- **注意点**
  - ここで見ているのは節点侵害であり、部材線分の完全交差判定ではありません。

### 5. 最適化設定

- **主な変数**
  - `config.W_mass`, `config.W_stiff`
  - `config.K_min`, `config.K_target`
  - `config.max_cg_z`
  - `config.exclusion_zones`
- **責務**
  - 目的関数・拘束・除外領域を `frameObjectiveFunction` が再利用できる形にまとめます。

### 6. 混合整数最適化

- **設計変数**
  - `x(1:5)`: パイプ群インデックス
  - `x(6:9)`: 形状パラメータ
  - `x(10:12)`: 距離閾値
  - `x(13:14)`: ノード切替
  - `x(15:18)`: ノード空間比
- **責務**
  - 形状・トポロジー・断面選択を同時に探索します。

## 前提・注意点

- コメントにもある通り、実際の荷重ケース生成は目的関数内部へ移っています。
- TCP FEM サーバ起動に依存するため、通信失敗時は最適化全体が止まります。
- 実行時間は長く、検証用デモというより研究・探索向けです。
