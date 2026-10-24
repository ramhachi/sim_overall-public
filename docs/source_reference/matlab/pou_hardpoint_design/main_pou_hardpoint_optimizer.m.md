# `matlab/pou_hardpoint_design/main_pou_hardpoint_optimizer.m`

## 概要
ダブルウィッシュボーン用POUハードポイント逆設計の中核関数です。JSONから車両情報を読み込み、幾何定義・初期値生成・`fmincon` によるマルチスタートSQP最適化・曲線評価・実装妥当性検証を統合します。

## 主要関数・クラス
- `main_pou_hardpoint_optimizer`: 最適化全体を統括する主関数
- `define_suspension_geometry`: 幾何モデル生成
- `objective_polynomial_fit`: 目標曲線追従コスト
- `constraint_rod_misalignment`: ロッドと実装制約
- `forward_kinematics`: ジャックアップ・キャンバー算出

## 理論・背景
車両レベル最適化で得た目標多項式を、実際に製作可能なハードポイントへ落とし込む逆問題を解きます。SQPで局所探索を行いながら複数初期値で多峰性に対応し、キングピン剛体運動・ジャックアップ・キャンバー・取付比・ロッド偏芯を同時に評価します。

## 外部ライブラリ・ツールボックス
- MATLAB Optimization Toolbox の `fmincon`
- MATLAB の JSON 処理
- 数値計算・行列演算機能

## 入出力
- 入力: `params_json_file`、`options`（`wheel_index`、`n_multistart` など）、ステア角評価範囲
- 出力: Mピボット座標、ベルクランク座標、多項式追従誤差、取付比、検証情報、関連曲線

## システム上の役割
POU設計ディレクトリ全体の中心であり、車両仕様と幾何ソルバ、制約関数、可視化、エクスポートを結び付ける統合点です。車両全体設計から具体ハードポイント設計へ進む際の主要ソルバとして使われます。
