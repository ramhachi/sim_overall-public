# `matlab/pou_hardpoint_design/export/export_hardpoints_csv.m`

## 1. 目的

最適化済み POU ハードポイントと参照用サスペンション点群を CSV に書き出す関数です。既定では SolidWorks の点群インポートに合わせたヘッダと列順で出力します。

## 2. 入出力と依存関係

- 入力:
  - `results`: `main_pou_hardpoint_optimizer` の結果構造体
  - `geom`: `define_suspension_geometry` の幾何構造体
  - `output_dir`: 出力先ディレクトリ（省略時 `.`）
  - `options`:
    - `wheel_name`（既定は `geom.wheel_index` から推定）
    - `format`（`solidworks` または `general`、既定 `solidworks`）
- 出力:
  - `csv_path`: 作成した CSV パス
- 依存関係:
  - ローカル関数 `write_point`
  - `mkdir`, `fopen`, `fprintf`, `fclose`

## 3. 全体処理フロー

1. 出力ディレクトリとオプション既定値を補います。
2. `pou_hardpoints_<wheel>.csv` というファイル名を決めます。
3. `results` と `geom` から chassis 座標系の点群を `points` 構造体に集めます。
4. kingpin は原点に加えて、方向ベクトルへ 0.15 m 延長した tip も生成します。
5. `solidworks` 形式なら mm 単位の 4 列、`general` 形式なら m と mm の両方を持つ 8 列で書き出します。
6. 保存完了後にパスを表示します。

## 4. 関数詳細

### `export_hardpoints_csv(results, geom, output_dir, options)`

- **収集する点**  
  M ピボット、ベルクランク、上下 A アーム取付点、タイロッド、キングピン原点・先端、キャリパ中心を出力対象にしています。

- **単位変換**  
  `solidworks` では各点を m→mm へ変換してから書き出します。

- **フォーマット分岐**  
  `solidworks` は `Point Name,X [mm],Y [mm],Z [mm]` のみ、`general` はホイール名と m/mm 両方を含めます。

### `write_point(fid, name, coords_mm)`

1 点分を `name,x,y,z` で 1 行出力する補助関数です。座標は mm 前提で受け取ります。
