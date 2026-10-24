# `matlab/rl_driver/exportActorForCpp.m`

## 役割

`exportActorForCpp` は、MATLAB RL Toolbox の学習済み Actor を C++ 実行系へ持ち込むために、**mean-path 重み・観測正規化情報・一致性確認用テストデータ**へ分解して書き出す関数です。

このファイルは deployment 直結の export 本体であり、`publishRLDeploymentCandidate` からも呼ばれます。

---

## 出力ファイル

### `rl_actor_weights.txt`

Actor の mean-path に属する全結合層重み・バイアス・LayerNorm パラメータを順番に保存します。  
確率分布の標準偏差側 `fcStd` は export 対象外です。

### `rl_normalization_params.txt`

観測 bounds と行動 scale を保存します。  
MATLAB の `FSAEEnvironment` と C++ 側で同じ正規化契約を使うための橋渡しです。

### `rl_actor_test_data.csv`

ランダムな正規化観測 100 点と、それに対する MATLAB Actor 出力を保存します。  
C++ 側で export 再現性を確認するための比較データです。

---

## export の全体フロー

1. 出力ディレクトリを作る。
2. `assertAgentObservationDimension` で Agent の観測次元を検証する。
3. Actor モデルを取り出す。
4. `getActionInfo(agent)` から行動次元を取得する。
5. `extractMeanPathLayers` で mean-path の FC 層列を抽出する。
6. `rl_actor_weights.txt` へ layer 単位で重み・バイアス・LayerNorm を書く。
7. `resolve_rl_vehicle_params` と `get_rl_observation_bounds` で正規化パラメータを計算する。
8. `rl_normalization_params.txt` を書く。
9. 乱数 seed 42 の 100 サンプルで Actor 出力を生成し、`rl_actor_test_data.csv` を書く。
10. 標準出力へ export 概要を表示する。

---

## 関数詳細

## `exportActorForCpp(agent, outputDir, vehicleParams)`

### 入力

| 引数 | 意味 |
| --- | --- |
| `agent` | 学習済み RL Agent |
| `outputDir` | export 出力先。省略時は `rl_results` |
| `vehicleParams` | 観測 bounds / action scale へ反映する車両パラメータ override |

### Actor 取得

まず `getActor(agent)` を呼び、`getModel(actor)` を試し、失敗時は `actor.Model` を使います。  
この点は `getDeterministicActorModel` と同じ互換方針です。

### 行動次元

`actInfo.Dimension(1)` を読みます。  
2 次元なら `steer + accel`、3 次元なら `steer + throttle + brake` とみなして normalization ファイルの行動スケールを書き分けます。

### 正規化パラメータ

- 観測 bounds は `get_rl_observation_bounds(exportVehicleParams)` を使用
- steering scale は `torque_mode` なら `T_driver_max`、そうでなければ `max_angle`
- アクセル / スロットル / ブレーキは 1.0 固定

ここで `resolve_rl_vehicle_params(vehicleParams)` を通すため、baseline テンプレートに対する部分 override がそのまま export 契約へ反映されます。

### テストデータ

`nTest = 100`、`rng(42)` 固定です。  
入力は各観測次元ごとに `[-1, 1]` の一様乱数で、これは「正規化済み観測空間のランダム点」を意味します。  
各点に対する Actor 出力を MATLAB 側で直接計算し、CSV に保存します。

---

## `extractMeanPathLayers(net)`

mean-path export の中核 helper です。  
`net.Layers` を先頭から走査し、全結合層だけを拾います。

### 抽出規則

- `FullyConnected` を含む層だけを見る
- 層名に `Std` を含むものは標準偏差パスとみなし除外する
- 直後に `LayerNormalization` があれば、その `Scale` と `Offset` を同じ entry に結び付ける
- そうでなければ LayerNorm なし FC 層として保存する

### 出力 struct

各 layer entry は以下を持ちます。

- `name`
- `W`
- `b`
- `ln_gamma`
- `ln_beta`

重要なのは、この helper は LayerNorm を数学的に折り畳んでいない点です。  
FC 層と、その直後に付く LayerNorm パラメータを**連続する層セットとして直列化**しています。

---

## `safeExtract(x)`

`dlarray` と通常数値配列の差を吸収して `double` 配列へ変換します。  
export する重み・バイアス・LayerNorm パラメータ・テスト出力のすべてで使われます。

---

## ファイルフォーマットの読み方

### `rl_actor_weights.txt`

layer ごとに以下の順で並びます。

1. `LAYER <name>`
2. `WEIGHTS rows cols`
3. 重み要素を行優先で列挙
4. `BIAS n`
5. バイアス列挙
6. LayerNorm があれば `LN_GAMMA`, `LN_BETA`

C++ 側はこの順番を前提に再構成します。

### `rl_normalization_params.txt`

1. `OBS_BOUNDS <nObs>`
2. 各観測の `[min max]`
3. `ACTION_SCALE <actDim>`
4. 行動スケール列挙

### `rl_actor_test_data.csv`

列は `obs_1 ... obs_N, act_*` です。  
MATLAB 出力との差分を C++ 側で比較する用途です。

---

## export internals の要点

- export 対象は **平均経路のみ**で、確率分布の分散パスは捨てる
- LayerNorm は消さずに別パラメータとして保存する
- 観測 bounds は `FSAEEnvironment` と共有契約を使う
- steering scale は torque mode / angle mode で切り替わる
- テストデータ生成に固定 seed を使い、再現比較を可能にしている
