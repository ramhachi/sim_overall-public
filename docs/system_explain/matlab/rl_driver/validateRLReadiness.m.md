# `matlab/rl_driver/validateRLReadiness.m`

## 1. 役割

`validateRLReadiness` は、学習済み RL ドライバーが**配備候補として最低限使えるか**を、複数トラック・複数エピソードで数値判定する関数です。  
単なる可視化ではなく、合格/不合格を返す **readiness validator** です。

判定軸はコメントにもある通り、次の 3 つです。

1. 完走率
2. コース逸脱率
3. ラップタイム（必要なら PID 基準比）

---

## 2. 入口関数 `summary = validateRLReadiness(agentOrPath, varargin)`

### 2.1 入力

- `agentOrPath`
  - agent オブジェクト、または agent 保存ファイルへのパスです。
- `varargin`
  - `inputParser` で解析されるオプション群です。

### 2.2 主なオプション

- `Tracks`
  - 評価対象トラック一覧。既定値は `{'autocross','skidpad','circle'}`。
- `Episodes`
  - 各トラックで試すエピソード数。
- `VehicleParams`
  - 環境に渡す車両パラメータ。
- `StartServer`
  - true なら評価前に `startRLServer()` を起動。
- `DomainRandStrength`
  - 0 より大きいと domain randomization を有効化。
- `DomainRandInterval`
  - randomization を適用する間隔。
- `MinCompletionRate`
  - 完走率の下限。
- `MaxOffCourseRate`
  - 許容される逸脱率の上限。
- `MaxLapTimeRatio`
  - PID 基準比の上限。
- `PIDBaselineLapTime`
  - `autocross/skidpad/circle` 等のフィールドを持つ基準 struct。
- `SaveJson`
  - 非空なら JSON レポートを保存。

### 2.3 出力 `summary`

返り値は readiness 判定結果の構造体です。主に次を含みます。

- `generated_at`
- `config`
- `tracks.<trackKey>`
- `overall_pass`

各トラック配下には、完走率・逸脱率・中央値ラップタイム・PID 比・合否詳細が入ります。

---

## 3. 主処理の流れ

### 3.1 agent 解決と deterministic actor 化

`agentOrPath` が文字列なら `loadRLAgentFromFile` で読み込みます。  
その後 `assertAgentObservationDimension` で観測次元を確認し、`getDeterministicActorModel` で決定論的 actor を得ます。

ここでも評価時は探索ノイズを使わず、**同じ観測に対し同じ行動を返す評価モード**に固定しています。

### 3.2 必要に応じた RL サーバ起動

`opts.StartServer` が true のとき `startRLServer()` を呼びます。  
このファイル自身はサーバ起動状態を細かく管理せず、「必要なら起動要求する」位置づけです。

### 3.3 トラックごとの評価ループ

各トラックで `FSAEEnvironment(trackName, opts.VehicleParams)` を生成し、`onCleanup` で削除予約します。  
さらに domain randomization が指定されていれば以下を環境へ設定します。

- `DomainRandEnabled = true`
- `DomainRandInterval`
- `DomainRandStrength`
- `DomainRandBounds = getVehicleParamBounds()`

これにより、固定条件だけでなく、多少ばらついた条件でも安定して走れるかを確認できます。

### 3.4 各エピソードの評価

1 エピソード内では次を繰り返します。

1. `reset(env)` で初期化
2. `predictDeterministicAction(actorModel, obs)` で行動生成
3. `step(env, action)` で 1 step 実行
4. 左右エッジ距離 `distLeft`, `distRight` と `env.OffTrackMargin` を見て off-course を検出
5. `done` または最大 step 到達で終了

エピソード終了後は次を記録します。

- `completedFlags(ep)` : `env.LapCompleted`
- `offCourseFlags(ep)` : エッジ接近または終了理由 `'offtrack'`
- `lapTimes(ep)` : 完走した場合だけ `stepCount * env.ControlDt`
- `terminationReasons(ep)` : 終了理由文字列

### 3.5 トラック単位の集計

各トラックごとに以下を計算します。

- `completionRate = mean(completedFlags)`
- `offCourseRate = mean(offCourseFlags)`
- `medianLap = median(validLapTimes)`
- `pidBaseline`
- `lapTimeRatio = medianLap / pidBaseline`

ラップタイム基準については分岐があります。

- PID 基準がある場合  
  `lapTimeRatio <= MaxLapTimeRatio` を要求
- PID 基準がない場合  
  `medianLap` が NaN でない、つまり少なくとも完走ラップが取れていることを要求

### 3.6 合否判定

トラック単位の合否は次の AND です。

- `completionPass`
- `offCoursePass`
- `lapPass`

全体合否 `summary.overall_pass` は、全トラックの `trackPass` を AND 累積したものです。

### 3.7 JSON 保存

`SaveJson` が指定されていれば `jsonencode(..., 'PrettyPrint', true)` で整形 JSON を保存します。  
外部ゲートや配備判定ログから機械的に読めるようにするための機能です。

---

## 4. トラック別サマリー `trackSummary`

各トラックについて、`summary.tracks.(trackKey)` には以下が入ります。

- `track`
- `episodes`
- `completion_rate`
- `off_course_rate`
- `median_lap_time`
- `pid_baseline_lap_time`
- `lap_time_ratio`
- `pass`
- `pass_detail`
  - `completion`
  - `off_course`
  - `lap_time`
- `termination_reason_counts`

`pass_detail` があるため、「なぜ fail したか」を機械的に追いやすい構造です。

---

## 5. ローカルヘルパー `cleanupEnv(env)`

### 5.1 役割

トラック評価のたびに生成した `FSAEEnvironment` を安全に破棄します。

### 5.2 処理

- 空でないことを確認
- `isvalid(env)` を確認
- `delete(env)` を実行

`onCleanup` から呼ばれることを前提にした、小さいが重要なリソース管理ヘルパーです。

---

## 6. ローカルヘルパー `countReasons(reasons)`

### 6.1 役割

各エピソードの終了理由配列 `reasons` を、**理由ごとの件数 struct** にまとめます。

### 6.2 処理内容

1. `unique(reasons)` でユニークな終了理由一覧を得る
2. 各理由について
   - 空文字列は無視
   - `matlab.lang.makeValidName` で struct フィールド名へ変換
   - 該当件数を `sum(reasons == uniqueReasons(i))` で数える

### 6.3 なぜ必要か

終了理由をそのまま配列で残すだけでは集計しづらいためです。  
この helper により、`offtrack`, `timeout`, `completed` などの傾向を JSON レポート上で即座に確認できます。

---

## 7. このファイルのシステム上の位置付け

`validateRLReadiness` は、RL 評価系の中で **定量ゲート判定** を担います。

- `evaluateRLDriver`  
  単発走行の可視化と目視確認
- `validateRLReadiness`  
  複数試行の数値判定
- `runRLReadinessGate`  
  actor export、C++ 整合確認、readiness 判定を束ねる上位ゲート

したがって本ファイルは、RL モデルを「学習済み」から「配備候補」へ進める際の中心判定器です。

---

## 8. 注意点

- 逸脱判定は `LastTerminationReason == 'offtrack'` だけでなく、走行中に一度でもエッジ余裕が `OffTrackMargin` 以下になったかでも検出します。
- ラップタイム統計は完走ラップのみを対象にし、未完走エピソードは `NaN` のまま除外します。
- `SaveJson` 先の親ディレクトリ作成はこの関数自身では行わないため、通常は呼び出し元で保存先を準備する設計です。
