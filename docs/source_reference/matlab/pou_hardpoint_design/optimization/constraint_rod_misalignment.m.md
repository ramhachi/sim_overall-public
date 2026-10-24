# `matlab/pou_hardpoint_design/optimization/constraint_rod_misalignment.m`

## 概要
`fmincon` 用の非線形不等式制約を返す関数です。ロッドの偏芯角、プッシュロッド長さ、取付比、Mピボット半径、各種クリアランスを含む8本の制約を扱います。

## 主要関数・クラス
- `constraint_rod_misalignment`: 制約計算の主関数
- 幾何評価処理: 順運動学結果に基づく制約計算

## 理論・背景
POU設計では目的曲線に合うだけでなく、機械要素として無理のないリンク姿勢と寸法であることが必要です。偏芯角や長さ・取付比の制限は、耐久性と組付け性を守るための実装制約として重要です。

## 外部ライブラリ・ツールボックス
- MATLAB Optimization Toolbox
- `forward_kinematics` などの幾何関数

## 入出力
- 入力: `design_vars`、`geom`、`delta_range`
- 出力: 不等式制約 `c [8x1]`、等式制約 `ceq []`

## システム上の役割
数値最適化が非現実的な解へ流れないようにする安全柵です。POUの物理成立性を探索中に常時監視する役割を担います。
