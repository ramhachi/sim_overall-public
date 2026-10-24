# `frame_optimize/examples/demo_full_pipeline.m`

## 役割

- 材料読込、フレーム定義、GA 実行、簡易 Pareto 表示、ビューア起動までを短時間で通すエンドツーエンドデモです。
- 本格運用よりも、フレーム最適化パイプライン全体の結線確認に向いています。

## スクリプトの流れ

- **1. データ読込**
  - 材料・パイプカタログ・簡易規則パラメータを準備します。
- **2. フレーム形状定義**
  - 実車寸法を使う分岐を持ちつつ、デフォルトは 16 ノード前後の簡略モデルです。
  - コックピット内を通らないレイアウトを手で定義します。
- **3. 境界条件と荷重**
  - 基本は 1000 Nm のねじり荷重 1 ケースです。
- **4. 最適化設定**
  - 質量重み、剛性重み、剛性閾値、GA 世代数などを低めに設定します。
- **5. 最適化実行**
  - `runFrameOptimization` を呼びます。
- **6. Pareto 風可視化**
  - 履歴そのものではなく、デモ用の擬似散布データから `extractParetoFront` を使って表示します。
- **7. `fsae_viewer` 起動**

## 入力

- `materials.json`
- `pipe_catalog.json`
- 任意で `real_vehicle_geometry.json`
- `suspension_base_geometry.json`
- `runFrameOptimization`, `extractParetoFront`, `fsae_viewer`

## 出力

- `best_x`, `best_fval`, `history`
- `pareto_results`
- ビューア表示

## セクション詳細

### 2. フレーム形状定義

- **出力**
  - `nodes`
  - `elements_geom.connectivity`
  - `elements_geom.group_indices`
  - `elements_geom.group_names`
- **責務**
  - 各要素を `front_hoop`, `main_hoop`, `side_impact`, `structural` のグループに分け、パイプ選択変数と結び付けます。
- **注意点**
  - コメントにある通り、コックピット外周を回る配置を意識しています。

### 3. 境界条件と荷重

- **入力**
  - 3 点拘束用 `bcs`
  - ねじり荷重 `loads`
- **注意点**
  - `use_multiple_loads` を有効にしても、実車寸法読み込み時にだけ 3 荷重ケースへ拡張する構成です。

### 4. 最適化設定

- **主なパラメータ**
  - `W_mass=1.0`
  - `W_stiff=0.5`
  - `K_min=300`
  - `K_target=800`
  - `pop_size=30`
  - `max_generations=5`
- **責務**
  - デモ時間を短く保ちながら最適化の流れだけ確認できる設定にします。

### 6. Pareto 表示

- **責務**
  - 単一目的最適化結果しかないため、擬似質量・剛性散布図を使ってフロント表示だけを再現します。
- **注意点**
  - この Pareto は解析結果そのものではなく、UI 表示用のダミーです。

## 前提・注意点

- C++ 後端 `beam_fem_standalone` がない場合、結果が NaN になる可能性があります。
- デモ性を優先しており、規則適合や最適性の保証は弱いです。
