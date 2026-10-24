# `frame_optimize/examples/test_real_vehicle_quick.m`

## 役割

- 実車向け JSON・荷重ケース生成・材料読込が最低限成立するかを、FEM を回さずに素早く確認するヘルスチェックです。

## スクリプトの流れ

- `real_vehicle_geometry.json` を読み、必須フィールドの存在を確認します。
- 車両寸法と除外ゾーン一覧を表示します。
- 4 ノードだけの簡易節点に対して `generate_load_cases` を呼び、荷重ケース構造を検証します。
- `loadMaterials`、`loadPipeCatalog` で材料・パイプ読込を確認します。
- 除外ゾーンの型と寸法データを検証して終了します。

## 入力

- `real_vehicle_geometry.json`
- `materials.json`
- `pipe_catalog.json`

## 出力

- JSON 構造検証ログ
- 荷重ケース一覧
- 材料・パイプカタログ件数

## セクション詳細

### geometry 読込検証

- **責務**
  - `dimensions`, `exclusion_zones`, `frame_nodes` が存在するか確認します。

### `generate_load_cases` 検証

- **入力**
  - 4 ノードの簡易節点列
- **出力**
  - 荷重ケース構造体配列
- **責務**
  - `name`, `loads`, `description` が揃っているか確認します。

### 除外ゾーン型検証

- **責務**
  - `type='AABB'`、`min/max` の 3 次元ベクトル、`name` の存在を assert で確認します。

## 前提・注意点

- あくまでデータ整合性確認であり、構造性能は評価しません。
