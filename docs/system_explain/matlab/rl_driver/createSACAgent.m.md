# `matlab/rl_driver/createSACAgent.m`

## 1. 役割

`createSACAgent` は、この RL 環境向けの Soft Actor-Critic エージェントを構築するファクトリ関数です。  
観測次元と行動次元を環境オブジェクトから取得し、Actor 1 本と Critic 2 本を定義して `rlSACAgent` を返します。

## 2. ファイル内の関数構成

1. `createSACAgent(env)` : エージェント全体の組み立て
2. `createCriticNetwork(obsDim, actDim)` : Q 関数ネットワーク生成
3. `countParams(net)` : 学習可能パラメータ総数の集計

## 3. `createSACAgent(env)` 詳細

まず `getObservationInfo(env)` と `getActionInfo(env)` から寸法を取得します。

- `obsDim` : 環境観測次元
- `actDim` : 行動次元。現行環境では 2

### Actor ネットワーク

Actor はガウス方策です。

共通幹:

- `featureInputLayer(obsDim)`
- `FC(256) -> LN -> ReLU`
- `FC(256) -> LN -> ReLU`
- `FC(128) -> LN -> ReLU`

分岐先:

- 平均出力 `fcMean -> tanh`
- 標準偏差出力 `fcStd -> softplus`

`tanh` により平均は行動範囲に収まりやすくなり、`softplus` により標準偏差は正値に保たれます。  
この `layerGraph` を `dlnetwork` 化し、`rlContinuousGaussianActor` として包みます。

### Critic ネットワーク

Critic は twin Q 構成です。

- `criticNet = createCriticNetwork(obsDim, actDim)` で 1 本目を生成
- 同じ関数をもう一度呼び、独立重みの 2 本目を生成
- それぞれ `rlQValueFunction` に変換

twin critic は Q 値の過大評価抑制が目的です。

### SAC オプション

`rlSACAgentOptions` に以下を設定します。

- `SampleTime = env.ControlDt`
- `DiscountFactor = 0.99`
- `TargetSmoothFactor = 5e-3`
- `ExperienceBufferLength = 1e6`
- `MiniBatchSize = 256`
- Actor optimizer 学習率 `3e-4`
- Critic optimizer 学習率 `3e-4`
- 勾配しきい値 `1.0`
- エントロピー温度学習率 `3e-4`
- 目標エントロピー `-actDim`

最後に `rlSACAgent(actor, [critic1, critic2], agentOpts)` を返し、概算パラメータ数を表示します。

## 4. `createCriticNetwork(obsDim, actDim)` 詳細

Q 関数用の `dlnetwork` を構築します。

構造:

- 観測入力 `featureInputLayer(obsDim, 'obs_in')`
- 行動入力 `featureInputLayer(actDim, 'act_in')`
- `concatenationLayer(1, 2)` で縦方向結合
- 以降は `FC(256) -> LN -> ReLU` を 2 段
- `FC(128) -> LN -> ReLU`
- `FC(1)` でスカラー Q 値出力

返り値は `rlQValueFunction` に渡せる `dlnetwork` です。

## 5. `countParams(net)` 詳細

`net.Learnables` テーブルを走査し、各 `Value{i}` の要素数を加算します。  
学習に直接関与する総パラメータ数の目安をログ表示するための補助関数です。

## 6. 実装上の要点

- 観測次元をハードコードせず環境から取得するため、観測契約変更への追従が容易です。  
- ただし学習済み重み自体は観測次元に依存するため、契約が変わったら再学習が必要です。  
- Actor と Critic の学習率、バッファ長、バッチサイズは本ファイルが SAC 学習の基本設定点になっています。
