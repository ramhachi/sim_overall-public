# `matlab/pou_hardpoint_design/optimization/`

## 概要
POUハードポイント設計における目的関数、制約、初期値生成をまとめた最適化モジュールです。車両目標曲線への追従と物理的成立性の両立を数式として表現します。

## 主要関数・クラス
- [`constraint_rod_misalignment.m.md`](constraint_rod_misalignment.m.md)
- [`generate_initial_guess.m.md`](generate_initial_guess.m.md)
- [`objective_polynomial_fit.m.md`](objective_polynomial_fit.m.md)

## 理論・背景
非線形最適化では、目的関数だけでなく初期値と制約が結果品質を大きく左右します。POU設計では、ジャックアップ追従、取付比、ロッド偏芯、長さ境界、クリアランスを同時に扱う必要があります。

## 外部ライブラリ・ツールボックス
- MATLAB Optimization Toolbox
- 幾何ソルバ群
- 基本統計量計算

## 入出力
- 入力: `geom`、目標多項式、設計変数、ステア角レンジ
- 出力: 初期推定値、目的関数値、非線形不等式制約

## システム上の役割
上位の `fmincon` が探索する評価空間そのものを定義する層です。POU設計の設計意図を数式へ落とし込む中心的な役目を持ちます。
