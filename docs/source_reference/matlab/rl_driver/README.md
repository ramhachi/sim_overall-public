# `matlab/rl_driver/`

## 概要
強化学習ドライバモデルの索引です。Soft Actor-Critic（SAC）を用いた自律走行エージェントの学習・評価・エクスポート・導入確認をMATLAB側で扱い、C++シミュレータとはTCP通信で接続します。

## 主要関数・クラス
- 中核:
  - [`FSAEEnvironment.m.md`](FSAEEnvironment.m.md)
  - [`FSAESimClient.m.md`](FSAESimClient.m.md)
  - [`createSACAgent.m.md`](createSACAgent.m.md)
  - [`trainRLDriver.m.md`](trainRLDriver.m.md)
  - [`evaluateRLDriver.m.md`](evaluateRLDriver.m.md)
  - [`exportActorForCpp.m.md`](exportActorForCpp.m.md)
  - [`benchmarkRLLaptimeSim.m.md`](benchmarkRLLaptimeSim.m.md)
- 学習スクリプト:
  - [`trainCircleSimple.m.md`](trainCircleSimple.m.md)
  - [`trainCircleV3.m.md`](trainCircleV3.m.md)
  - [`trainCircleV4.m.md`](trainCircleV4.m.md)
  - [`trainAutocrossV6.m.md`](trainAutocrossV6.m.md)
  - [`trainMultiTrackV5.m.md`](trainMultiTrackV5.m.md)
  - [`trainCurriculumV7.m.md`](trainCurriculumV7.m.md)
  - [`trainV8.m.md`](trainV8.m.md)
  - [`trainV9.m.md`](trainV9.m.md)
  - [`trainV10.m.md`](trainV10.m.md)
  - [`trainV10_finetune.m.md`](trainV10_finetune.m.md)
  - [`trainWithCurriculum.m.md`](trainWithCurriculum.m.md)
  - [`trainAceDriverPhase4Extension.m.md`](trainAceDriverPhase4Extension.m.md)
  - [`resumeAceDriverPhase4ExtensionAfterRestart.m.md`](resumeAceDriverPhase4ExtensionAfterRestart.m.md)
- 補助:
  - [`rl_driver_quickstart.m.md`](rl_driver_quickstart.m.md)
  - [`runRLReadinessGate.m.md`](runRLReadinessGate.m.md)
  - [`startRLServer.m.md`](startRLServer.m.md)
  - [`validateRLReadiness.m.md`](validateRLReadiness.m.md)
  - [`ensure_rl_driver_paths.m.md`](ensure_rl_driver_paths.m.md)
  - そのほか観測・乱数化・推論補助の各 `*.m.md`

## 理論・背景
SACはエントロピー最大化を含むオフポリシー連続制御法であり、操舵と加減速を同時に扱う車両制御に適しています。本ディレクトリでは、34次元観測、進捗・速度・安全性・滑らかさを組み合わせた報酬、TCP越しのC++物理シミュレータ、ドメインランダム化、学習済み方策のC++輸出を一つの流れにまとめています。

## 外部ライブラリ・ツールボックス
- MATLAB Reinforcement Learning Toolbox
- MATLAB Deep Learning Toolbox
- Java `java.net.Socket`
- Docker経由のC++ RLサーバ

## 入出力
- 入力: トラック名、車両パラメータ、学習オプション、TCP接続設定、学習済みエージェント
- 出力: 学習済みSACエージェント、評価結果、軌跡・時系列図、C++推論用重みファイル、準備完了レポート

## システム上の役割
このディレクトリは、MATLAB上の学習アルゴリズムとC++上の高忠実度車両モデルをつなぐ制御知能レイヤです。研究段階の学習実験から、推論モデルのエクスポートと導入可否判定までを統合して扱います。
