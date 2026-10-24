# `matlab/rl_driver/evaluateRLDriver.m`

## 1. 役割

`evaluateRLDriver` は、学習済み RL ドライバーを 1 エピソード実走させ、**性能確認・可視化・保存** をまとめて行う評価用エントリです。  
主目的は「その agent が実際にどう走るか」を、人が図で確認できる状態にすることです。

この関数は学習そのものは行わず、以下を担当します。

- agent の読み込みまたは受け取り
- `FSAEEnvironment` の生成
- 決定論的 policy による 1 エピソード走行
- 観測・操作・報酬・車速・偏差のログ取得
- 6 分割の評価図作成
- `rl_results/<track>/evaluation.png` への保存

---

## 2. 入口関数 `evaluateRLDriver(agentOrPath, trackName, vehicleParams)`

### 2.1 入力

- `agentOrPath`
  - 学習済み agent オブジェクト、または `.mat` へのパスです。
  - 文字列なら `loadRLAgentFromFile` で読み込みます。
- `trackName`
  - 評価対象トラック名です。
  - 未指定時は `'circle'` です。
- `vehicleParams`
  - 環境生成時に渡す車両パラメータ構造体です。
  - 未指定時は空 struct です。

### 2.2 この関数が返すもの

戻り値はありません。  
代わりに、標準出力へのサマリー表示・Figure 表示・PNG 保存を行います。

### 2.3 主要処理

#### (1) agent の解決

入力がパス文字列なら `loadRLAgentFromFile` を呼び、agent を取り出します。  
その後 `assertAgentObservationDimension` で観測次元の整合性を確認し、`getDeterministicActorModel` で**平均行動のみを返す評価用 actor** を取得します。

ここで決定論的 actor に変換している理由は、学習時の探索ノイズを評価時に持ち込まず、再現性の高い挙動確認を行うためです。

#### (2) 環境生成と後始末予約

`FSAEEnvironment(trackName, vehicleParams)` を生成し、`onCleanup(@() cleanupEnv(env))` で終了時の削除を予約します。  
これにより、途中で error が起きても環境インスタンスが残りにくくなっています。

#### (3) ログ配列の事前確保

`env.MaxEpisodeSteps` を上限として以下を事前確保します。

- `log.obs`
- `log.action`
- `log.reward`
- `log.vx`
- `log.yawRate`
- `log.lateralDev`
- `log.lapProgress`

MATLAB では動的伸長より事前確保の方が安定して速いため、評価ループの実装として妥当です。

#### (4) 1 エピソード評価ループ

各 step で次を行います。

1. `predictDeterministicAction(actorModel, obs)` で行動生成
2. `step(env, action)` で環境を 1 ステップ進める
3. 観測・報酬・ログ信号を `log` に保存
4. `totalReward` を累積
5. `isDone` なら終了

この関数は複数エピソード統計ではなく、**単発の走行ログ可視化** に特化しています。

#### (5) ログのトリム

事前確保した配列は常に最大長なので、実際に使った `nSteps` まで切り詰めます。  
これにより、未使用のゼロ埋め部分が可視化や後続計算に混ざりません。

#### (6) サマリー表示

以下をコンソールへ出力します。

- トラック名
- 実行ステップ数
- 経過時間 `nSteps * env.ControlDt`
- 総報酬
- 平均報酬
- ラップ完走フラグ
- 終了理由

完走したかだけでなく、どういう理由で終わったかを確認できるため、学習済み agent の健全性チェックに向いています。

#### (7) 可視化

Figure 上に 6 つのサブプロットを並べます。

1. **Control Inputs**
   - `action(1)` を steer として表示
   - `action(2)` は正を throttle、負を brake に分解して表示
2. **Vehicle Speed**
   - `log.vx` を km/h に変換して表示
3. **Lateral Deviation**
   - コース中心からの横偏差
4. **Reward**
   - 瞬時報酬と累積平均報酬
5. **Lap Progress**
   - ラップ進行率を %
6. **Steering vs Yaw Rate**
   - 操舵角とヨーレートの散布図
   - 色で時間進行を表現

特に 1 番と 6 番は、「どう操作して、車両がどう応答したか」を見やすくする意図があります。

#### (8) 保存

保存先は `rl_results/<trackName>/evaluation.png` です。  
保存先ディレクトリがなければ `mkdir` で作成します。

---

## 3. ローカルヘルパー `cleanupEnv(env)`

### 3.1 役割

環境オブジェクトを安全に破棄するための後始末ヘルパーです。

### 3.2 処理内容

- `env` が空でないこと
- `isvalid(env)` で有効なハンドルであること

を確認してから `delete(env)` を呼びます。

### 3.3 なぜ必要か

`FSAEEnvironment` がハンドルクラスである前提では、削除漏れにより内部状態や外部接続が残る可能性があります。  
`onCleanup` と組み合わせることで、正常終了・異常終了のどちらでもクリーンアップしやすくしています。

---

## 4. このファイルの設計上の意味

このファイルは、RL 学習パイプラインの中で **「訓練後の見える化」担当** です。  
`validateRLReadiness` が複数エピソードの合否判定を行うのに対し、こちらは 1 本の走行を詳しく観察する用途に向いています。

つまり役割分担は次の通りです。

- `evaluateRLDriver` : 挙動を人が見る
- `validateRLReadiness` : 配備基準を数値で判定する
- `runRLReadinessGate` : export/C++ 整合/readiness を通しで実行する

---

## 5. 注意点

- `Avg Reward/Step` は `nSteps` で割るため、理論上 `nSteps == 0` の異常系には弱いです。
- 先頭コメントには PID 比較表や軌跡境界付きプロットへの言及がありますが、現実装は主に 6 分割の時系列・散布図可視化です。
- `action(2)` を throttle/brake に分解する前提は、「縦方向操作が 1 軸の signed 値」という action 設計に依存します。
