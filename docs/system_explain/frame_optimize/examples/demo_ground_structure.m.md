# `frame_optimize/examples/demo_ground_structure.m`

## 役割

- 必須ノードと追加候補ノードから `generateGroundStructure` を使って候補部材集合を生成し、阻止領域付きで可視化するデモです。
- トポロジー最適化の前段である「候補辺生成」が何をしているかを確認するためのスクリプトです。

## スクリプトの流れ

- **1. データ読込**  
  材料・パイプカタログをロードします。
- **2. 基本ノード定義**  
  Front Hoop と Main Hoop を必須ノード、途中補剛点を追加候補ノードとして定義します。
- **3. 阻止領域定義**  
  コックピットとエンジンを `AABB` で登録します。
- **4. Ground Structure 生成**  
  `config.max_length_mm` を与えて候補部材を生成します。
- **5. 可視化**  
  左に候補全部材、右に必須構造のみを描画します。

## 入力

- `materials.json`
- `pipe_catalog.json`
- スクリプト内のノード座標
- `config.exclusion_zones`
- `config.max_length_mm = 2000`

## 出力

- `candidate_elements`
- 候補数の集計表示
- 3D 図（候補構造と必須構造）

## 依存関係

- `loadMaterials`
- `loadPipeCatalog`
- `generateGroundStructure`

## セクション詳細

### 基本ノード定義

- **責務**
  - 最低限必要なフープ 8 点と、最適化で追加可能な補剛点 10 点を分けて定義します。
- **設計意図**
  - 必須構造と任意構造を分離し、後段 GA で選択しやすくしています。

### 阻止領域定義

- **入力**
  - `type`, `min_bounds`, `max_bounds`, `name`
- **責務**
  - 部材候補が通ってはいけない領域を Ground Structure 生成器へ渡します。
- **注意点**
  - この例では `AABB` のみ使用しています。

### Ground Structure 生成

- **入力**
  - 全ノード、最大部材長、阻止領域
- **出力**
  - 候補部材の接続表 `candidate_elements`
- **注意点**
  - 全組み合わせから、長さ制約や阻止領域交差に反するものが除外されます。

### 可視化

- **責務**
  - 候補生成が意図通りか目視確認することです。
- **表示内容**
  - 候補部材を灰色、必須ノードを黒、阻止領域を半透明赤で描いています。

## 前提・注意点

- 解析や最適化は行わず、候補生成までです。
- 候補辺の品質は `generateGroundStructure` の交差判定実装に依存します。
