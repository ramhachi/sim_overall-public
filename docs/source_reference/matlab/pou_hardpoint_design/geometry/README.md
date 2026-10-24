# `matlab/pou_hardpoint_design/geometry/`

## 概要
POU機構の幾何計算を担う中核モジュール群です。ダブルウィッシュボーンの既存ハードポイント、キングピン周りの剛体運動、ベルクランク実体化、パッケージ判定まで、運動学の基盤を提供します。

## 主要関数・クラス
- [`axis_angle_rotation.m.md`](axis_angle_rotation.m.md)
- [`define_suspension_geometry.m.md`](define_suspension_geometry.m.md)
- [`forward_kinematics.m.md`](forward_kinematics.m.md)
- [`generate_physical_bellcrank.m.md`](generate_physical_bellcrank.m.md)
- [`solve_upright_position_3d.m.md`](solve_upright_position_3d.m.md)
- [`validate_packaging.m.md`](validate_packaging.m.md)

## 理論・背景
POU最適化の目的関数や制約は、最終的にはすべて幾何ソルバの結果に依存します。キングピン軸周りの回転、アップライトの剛体変換、プッシュロッド長変化、ベルクランク配置可能性がここで数値化されます。

## 外部ライブラリ・ツールボックス
- MATLAB 行列演算
- 基本三角関数・幾何計算
- JSON由来の車両寸法データ

## 入出力
- 入力: 車両パラメータ、輪番号、設計変数、ステア角レンジ
- 出力: 幾何構造体、ジャックアップ曲線、キャンバー曲線、アップライト状態、パッケージ違反情報

## システム上の役割
最適化層と可視化層の共通基盤として機能します。POU設計の物理的意味を定義し、上位処理が参照する標準運動学モデルを提供します。
