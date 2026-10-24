# `matlab/rl_driver/FSAEEnvironment.m`

## 概要
Reinforcement Learning Toolbox準拠のFSAE用環境クラスです。`rl.env.MATLABEnvironment` を継承し、C++シミュレータとのTCP接続、34次元観測生成、報酬計算、エピソード終了判定、ドメインランダム化を管理します。

## 主要関数・クラス
- `FSAEEnvironment`: 環境クラス本体
- `step`: 行動適用、シミュレータ更新、報酬計算
- `reset`: 新規エピソード初期化

## 理論・背景
RL環境は、エージェントが観測・行動・報酬の閉ループを学ぶための定義そのものです。本クラスは進捗、速度、安全性、操作の滑らかさを報酬へ反映し、さらにパラメータランダム化で過学習を抑えます。

## 外部ライブラリ・ツールボックス
- MATLAB Reinforcement Learning Toolbox
- TCP通信補助として `FSAESimClient`
- MATLAB クラスシステム

## 入出力
- 入力: トラック名、車両パラメータ、行動 `[steer, accel]`
- 出力: 観測ベクトル、報酬、終了フラグ、補助情報

## システム上の役割
MATLABの学習器とC++物理シミュレータを接続する最重要ブリッジです。学習実験の再現性と環境定義の一貫性をここで担保します。
