# `matlab/rl_driver/assertAgentObservationDimension.m`

## 1. 役割

`assertAgentObservationDimension` は、読み込んだ RL エージェントの観測次元が現行環境契約と一致しているかを検証するガード関数です。  
古い観測仕様で学習したエージェントを誤って再利用する事故を防ぎます。

## 2. `assertAgentObservationDimension(agent, expectedDim, context)` 詳細

処理の流れ:

1. `expectedDim` 未指定なら `size(get_rl_observation_bounds(), 1)` を使う
2. `context` 未指定なら `'Agent'` を使う
3. `getObservationInfo(agent)` からエージェントの観測 spec を取得
4. `obsInfo.Dimension(1)` を現在の観測次元として読む
5. `expectedDim` と違えば詳細エラーメッセージを投げる

エラーメッセージでは「旧観測契約の legacy agent は scratch から再学習が必要」と明示しています。

## 3. 実装上の要点

- 現行系では `get_rl_observation_bounds()` に基づく 45 次元観測が前提です。  
- 単なる warning ではなく error にしているため、不整合なエージェントは実行前に止まります。  
- `context` を差し込めるので、呼び出し側が「評価用 agent」「再開学習 agent」など識別しやすいメッセージを出せます。
