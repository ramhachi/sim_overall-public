# `matlab/pou_hardpoint_design/optimization/generate_initial_guess.m`

## 概要
マルチスタート最適化向けに複数の初期値戦略を生成する関数です。`nominal`、`conservative`、`aggressive`、`random` の各方針で探索開始点を与えます。

## 主要関数・クラス
- `generate_initial_guess`: 初期値生成の主関数
- 戦略分岐: 方針ごとのベクトル構築

## 理論・背景
POU設計は多峰性を持ちやすく、初期値次第で収束先が変わります。設計意図に沿った複数戦略を準備することで、`fmincon` のマルチスタート探索効率と安定性を高めます。

## 外部ライブラリ・ツールボックス
- MATLAB 乱数生成
- 幾何境界情報

## 入出力
- 入力: `geom`、`target_poly`、`strategy`
- 出力: 初期設計変数 `x0 [6x1]`

## システム上の役割
最適化本体の収束品質を底上げする前処理です。複数局所解が存在する問題で、再現性と探索多様性を両立させます。
