# `matlab/rl_driver/trainRLDriver.m`

## 概要
RLドライバ学習の主入口です。入力オプションを解釈し、環境構築、SACエージェント生成、学習設定、保存先管理、`rl.util.train` の実行を統合します。

## 主要関数・クラス
- `trainRLDriver`: 学習全体の主関数
- `FSAEEnvironment`: 環境生成先
- `createSACAgent`: エージェント生成先
- `applyRLEnvOverrides`: 環境上書き設定

## 理論・背景
性能の良いRLモデルは、ネットワーク構造だけでなく学習実験の設計に強く依存します。本関数は報酬設定、停止条件、再開学習、保存規則を一元化し、実験の再現性を確保します。

## 外部ライブラリ・ツールボックス
- MATLAB Reinforcement Learning Toolbox
- MATLAB Deep Learning Toolbox
- 学習ログ保存機能

## 入出力
- 入力: トラック名、車両パラメータ、`MaxEpisodes` や `StopReward` を含む学習オプション
- 出力: 学習済みエージェント、学習統計、保存済み成果物

## システム上の役割
RLディレクトリ全体の統合入口であり、実験実行の標準APIです。個別スクリプトや版管理された学習条件も最終的にはこの流れに収束します。
