# `frame_optimize/examples/visualize_suspension_geometry.m`

## 役割

- 懸架 JSON をフレーム座標系へ変換し、前後 A アーム・アップライト・ホイール中心を 3D で確認する可視化スクリプトです。

## スクリプトの流れ

- `suspension_base_geometry.json` と `optimized_autocross.json` を読みます。
- フロント用 `map_front`、リア用 `map_rear` の座標変換関数を定義します。
- FL を基準に FR を Y ミラーで生成し、RL を基準に RR を生成します。
- 変換後座標をコンソールへ表示します。
- `draw_arm` を使って前後上下の A アームを描き、アップライト・ホイール中心・タイヤ円を可視化します。

## 入力

- `suspension_base_geometry.json`
- `matlab/optimized_autocross.json`

## 出力

- 前後懸架のフレーム座標
- 3D 可視化図

## 主要セクション

### 座標変換

- **`map_front`**
  - 前輪基準座標をフレーム座標系へ変換します。
- **`map_rear`**
  - 後輪側はホイールベース `WB` を X へ加算して rear axle 基準から車体基準へ変換します。

### 点群抽出

- **責務**
  - chassis mount、upright、wheel center をそれぞれ前後左右に整理します。
- **注意点**
  - 右側は左側データの Y 絶対値ミラーで作っています。

### `draw_arm`（無名関数）

- **入力**
  - 前後 2 つのシャシーマウント点、1 つのアップライト点、色
- **出力**
  - 三角形状の A アーム線分描画

## 前提・注意点

- 左右対称を仮定しています。
- 視覚確認が目的で、力学解析は行いません。
