# `matlab/pou_hardpoint_design/export/export_hardpoints_csv.m`

## 概要
POU最適化で得たハードポイントをSolidWorks互換CSVなどの形式で書き出す関数です。座標は主にミリメートルへ変換し、CAD側で読み込みやすい列構成に整えます。

## 主要関数・クラス
- `export_hardpoints_csv`: CSV出力の主関数
- `options.format`: `solidworks` と `general` の切替点

## 理論・背景
CAD連携では単位系と列順が重要で、解析用座標をそのまま渡すと設計現場で扱いにくくなります。POU機構では左右輪・各ピン位置の明示が必要なため、規則的な表形式出力が実務上有効です。

## 外部ライブラリ・ツールボックス
- MATLAB のファイル書き込み機能
- CSV整形処理
- SolidWorks等CADツールを想定した交換フォーマット

## 入出力
- 入力: `results`、`geom`、`output_dir`、`options.format`
- 出力: CAD取込用CSVファイル

## システム上の役割
最適化結果を製作・設計の現場へ直接渡すための出口です。幾何計算の成果を外部CAD作業へ移す実務的な変換層として機能します。
