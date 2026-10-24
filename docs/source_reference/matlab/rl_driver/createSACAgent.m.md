# `matlab/rl_driver/createSACAgent.m`

## 概要
SACエージェントを構築するファクトリ関数です。アクタは全結合256ユニットを基調とした3層構成で平均と分散を出力し、クリティックは2本のQ関数を持つ双対構成を採用します。

## 主要関数・クラス
- `createSACAgent`: SAC生成の主関数
- アクタネットワーク構築部
- 2本のクリティック構築部

## 理論・背景
SACは報酬最大化と方策エントロピー最大化を同時に行うため、探索の頑健性が高い手法です。平均にTanh、分散にSoftplusを使う構成は、連続制御で安定した行動分布を作る典型です。

## 外部ライブラリ・ツールボックス
- MATLAB Reinforcement Learning Toolbox
- MATLAB Deep Learning Toolbox
- `rlContinuousGaussianActor`
- `rlQValueFunction`

## 入出力
- 入力: 観測仕様、行動仕様、ハイパーパラメータ
- 出力: 学習可能なSACエージェント

## システム上の役割
学習器の中身を定義する中核関数です。環境が「何を学ぶか」を決めるのに対し、この関数は「どう学ぶか」のモデル構造を定義します。
