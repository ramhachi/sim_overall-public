# `profile_rl_bridge.m`

> **要約**: RL（強化学習）サーバーとの TCP 通信および RL 環境のステップ性能を計測するベンチマークスクリプト。

## ファイルの目的

C++ RL サーバーへの TCP 接続、`reset` / `step` 操作のレイテンシを計測し、1 エピソード（3000 ステップ）の推定実行時間を算出する。RL 訓練のボトルネック分析と性能要件の検証に使用する。

## 計測項目

| 項目 | 回数 | 説明 |
|:---|:---|:---|
| TCP Client Reset | 100 回 | `FSAESimClient.reset()` の平均レイテンシ |
| TCP Client Step | 1000 回 | `FSAESimClient.step(0, 0.5, 0)` の平均レイテンシ |
| RL Env Reset | 100 回 | `FSAEEnvironment.reset()` の平均レイテンシ |
| RL Env Step | 1000 回 | `FSAEEnvironment.step([0; 0.5; 0])` の平均レイテンシ |

## 理論的背景

強化学習では数千〜数万エピソードの試行が必要で、1 ステップのレイテンシが訓練時間を支配する。TCP 通信のオーバーヘッドが環境ステップに対して大きい場合、共有メモリや gRPC への移行を検討する判断材料になる。

## 使用する MATLAB クラス

| クラス | 目的 |
|:---|:---|
| `FSAESimClient` | C++ RL サーバーへの TCP クライアント |
| `FSAEEnvironment` | RL 環境ラッパー（OpenAI Gym 互換インターフェース） |

## 入出力

- **入力**: RL サーバーが `localhost` の指定ポートで起動済みであること
- **出力**: 標準出力にレイテンシと推定エピソード時間を表示

## システム上の役割

`EasyStart.m` の RL ドライバー管理メニューの一部。訓練パイプラインの性能評価ツール。

## 実行方法

```matlab
>> profile_rl_bridge
```
