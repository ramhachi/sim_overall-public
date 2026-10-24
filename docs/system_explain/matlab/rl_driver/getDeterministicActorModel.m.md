# `matlab/rl_driver/getDeterministicActorModel.m`

## 役割

`getDeterministicActorModel` は、SAC の確率的 Actor から **deployment 用の deterministic mean-path モデル**を取り出す helper です。  
推論時にサンプリングを使わず、平均経路のみで行動を出したいときの入口になります。

`exportActorForCpp` や `predictDeterministicAction` と同じ思想で、`fcStd` 側ではなく Actor の mean path を基準に扱うための補助関数です。

---

## 関数詳細

## `getDeterministicActorModel(agent)`

### 処理手順

1. `assertAgentObservationDimension(agent)` で観測次元が現行契約と一致するか確認する。
2. `getActor(agent)` で RL Toolbox の Actor オブジェクトを取得する。
3. まず `getModel(actor)` を試す。
4. 失敗したら後方互換として `actor.Model` を返す。

### なぜ try/catch があるか

MATLAB RL Toolbox のバージョンや Actor オブジェクト実装差で、モデル取得 API が `getModel(actor)` の場合と `actor.Model` の場合があります。  
この関数はその差を吸収して、呼び出し側に dlnetwork 相当を返すことを優先しています。

### 観測次元チェックの意味

このリポジトリの RL 観測は vehicle-conditioned な現行契約を前提にしています。  
古い低次元 Agent を誤って export / deployment へ回さないため、取得前に `assertAgentObservationDimension` を必ず通しています。

### 返り値

返るのは Actor の学習済みモデル本体で、通常は `dlnetwork` です。  
このモデルを `predictDeterministicAction` に渡せば、確率サンプリング無しで平均行動を評価できます。

---

## 使いどころ

- MATLAB で deterministic 推論をしたいとき
- C++ export の前に mean-path モデルを直接確認したいとき
- Actor オブジェクト API 差を吸収して下流を単純化したいとき
