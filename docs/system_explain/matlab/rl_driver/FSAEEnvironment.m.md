# `matlab/rl_driver/FSAEEnvironment.m`

## 1. 役割

`FSAEEnvironment` は RL Toolbox の `rl.env.MATLABEnvironment` を継承した、FSAE 14DOF シミュレータ用の強化学習環境クラスです。  
MATLAB 側の 2 次元行動を、Docker 内 C++ RL サーバーが理解する 3 入力 `steer / throttle / brake` に変換し、返ってきた生観測を 45 次元の正規化観測へ整形します。

このクラスは単なるラッパーではなく、以下を一箇所で担っています。

- 観測空間・行動空間の定義
- TCP クライアント `FSAESimClient` の所有
- エピソードの初期化とリセット
- 報酬関数の計算
- 終了条件の判定
- Domain Randomization
- rolling start による初速付与
- ログ信号の生成

## 2. 観測と行動の契約

### 行動空間

行動は 2 次元です。

1. `steer_command` : `[-1, 1]`
2. `accel` : `[-1, 1]`

`accel >= 0` はスロットル、`accel < 0` はブレーキに変換されます。  
この設計により、アクセルとブレーキの同時入力を行動空間レベルで禁止しています。

### 観測空間

サーバー由来の生観測 34 要素に、車両諸元 11 要素を後段で連結し、合計 45 要素を `[-1, 1]` に正規化して返します。  
正規化レンジ自体は `get_rl_observation_bounds()` が管理します。

## 3. プロパティ詳細

### シミュレーション設定

- `ControlDt` : 制御周期。既定 0.02 秒
- `DeltaMax` : 角度操舵モード時の最大タイヤ舵角
- `MaxEpisodeSteps` : エピソード打ち切り上限
- `TrackName` : 現在のトラック名
- `TrackLength` : サーバー初期化後に得るトラック全長

### 報酬・判定関連の係数

- `W_progress` : 進行距離報酬
- `W_speed` : 速度ボーナス
- `W_boundary` : トラック端接近ペナルティ
- `W_centering` : センターライン追従報酬
- `W_slip` : 過大スリップ抑制
- `W_spin` : スピン抑制
- `W_jerk` : 操舵の急変抑制
- `W_lap` : ラップ完了ボーナス
- `W_heading` : ヘディング誤差抑制
- `W_alive` : 生存ボーナス
- `W_lowSpeedBrake` : 低速時にブレーキへ逃げる行動の抑制
- `W_previewSteer` : 先読み曲率と操舵方向の整合性報酬
- `TrackEdgePenaltyStartRatio` : 境界ペナルティ開始比率
- `OffTrackMargin` : オフトラック判定余裕
- `StallMinSpeed` / `StallGraceSteps` : 失速判定条件
- `CrawlWindowSteps` / `CrawlMinProgress` : 微小前進で stall を回避する抜け道対策
- `LowSpeedBrakeThreshold` : 低速ブレーキ罰則開始速度
- `PreviewSteerSpeedThreshold` / `PreviewCurvatureThreshold` : 先読み操舵報酬の適用条件
- `RollingStartSpeed` / `RollingStartThrottle` / `RollingStartMaxSteps` : rolling start の目標
- `RollingStartHoldEpisodes` / `RollingStartFadeEpisodes` : rolling start の保持・漸減期間

### TCP 接続設定

- `ServerHost` : サーバーホスト
- `ServerPort` : サーバーポート

### Domain Randomization

- `DomainRandEnabled` : 有効化フラグ
- `DomainRandInterval` : 何エピソードごとに再サンプルするか
- `DomainRandStrength` : 範囲中央からどれだけ広く振るか
- `DomainRandBounds` : `getVehicleParamBounds()` が返す境界構造体
- `EpisodeIndexOffset` : カリキュラム全体でのグローバル episode オフセット
- `TrackPool` : トラック群。空でなければ reset 時にランダム選択

### 車両パラメータ

- `BaseVehicleParams` : 基準車両
- `CurrentVehicleParams` : 現在エピソードに適用中の車両

### 読み取り専用の実行状態

- `Client` : `FSAESimClient` インスタンス
- `StepCount` : 現エピソードの経過ステップ数
- `PrevAction` : 1 ステップ前の行動
- `PrevS` : 1 ステップ前の周回距離 `s`
- `LapCompleted` : ラップ完了フラグ
- `EpisodeCount` : 環境単位の episode カウンタ
- `LastTerminationReason` : 最新終了理由
- `RecentProgressHistory` : crawl 判定用の進捗履歴

## 4. ライフサイクル

1. コンストラクタで観測・行動仕様と TCP 接続を初期化  
2. `reset()` で必要ならトラック変更・Domain Randomization・rolling start を実施  
3. `step()` で行動変換、サーバー実行、報酬計算、終了判定、正規化観測返却  
4. エピソード終了後は再び `reset()` へ戻る  
5. オブジェクト破棄時に `delete()` がソケットを閉じる

## 5. 公開メソッド詳細

### `FSAEEnvironment(trackName, varargin)`

環境生成の中心です。

主な処理:

1. `addpath` で関連 MATLAB 関数群を利用可能にする
2. `resolve_rl_vehicle_params()` で基準車両を取得
3. `varargin{1}` が構造体なら、その上書きを基準車両へ反映
4. `get_rl_observation_bounds(baseVehicleParams)` から観測次元を決定
5. 観測 spec を `[-1,1]` 固定で作成
6. 行動 spec を 2 次元 `[-1,1]` で作成
7. 親クラス `rl.env.MATLABEnvironment` を初期化
8. `ServerPort = get_rl_server_host_port()` を設定
9. `FSAESimClient` を作成して接続
10. 基準車両を JSON 化して `Client.init(trackName, paramsJson)` を呼び、トラック長を取得

この時点で環境は即座にシミュレータへ接続済みの状態になります。

### `[obs, reward, isDone, loggedSignals] = step(obj, action)`

学習ループの 1 ステップを実行します。

処理順:

1. `decodeSteeringAction(action(1))` で操舵次元を実入力へ変換
2. `action(2)` を throttle/brake のどちらか一方へ分配
3. `Client.step(steerInput, throttle, brake)` で C++ サーバーを 1 ステップ進める
4. `StepCount` を更新
5. `computeProgressDelta(info(4), info(1))` で進捗差分 `ds` とラップ跨ぎを検出
6. `recordRecentProgress(ds)` で crawl 判定履歴を蓄積
7. `checkTermination(info, lapJustCompleted)` で終了判定
8. `computeReward(...)` で報酬を算出
9. ラップ完了フラグや終了理由を更新
10. `normalizeObservation(rawObs)` で 45 次元正規化観測へ変換
11. 可視化・ログ用 `loggedSignals` を整形
12. `PrevAction` と `PrevS` を更新

`loggedSignals` には速度、ヨーレート、横偏差、境界までの距離、報酬、操舵入力などの可観測情報が入ります。

### `obs = reset(obj)`

新エピソード開始処理です。

主な処理:

1. `EpisodeCount` を増やし、`EpisodeIndexOffset` を足してグローバル episode 番号を作る
2. `TrackPool` があればトラックをランダム選択し、変更時は再初期化フラグを立てる
3. Domain Randomization 条件を満たす場合、`generateRandomVehicleParams()` で新車両を生成
4. 車両やトラックが変わった場合は `Client.init()` を再実行
5. `Client.reset()` でサーバー状態を初期化
6. `getEffectiveRollingStart()` の結果に応じて `applyRollingStart()` を実行
7. 生観測を正規化して返す
8. `StepCount`、`PrevAction`、`PrevS`、`LapCompleted`、`LastTerminationReason`、履歴を初期化

`PrevS` は rolling start 中に進んだ距離を考慮するため、必要に応じて 0 以外で開始されます。

### `delete(obj)`

`Client` があれば `disconnect()` して TCP 接続を閉じます。

## 6. 非公開メソッド詳細

### `computeReward(obj, rawObs, info, action, ds, lapJustCompleted, terminationReason)`

報酬設計の中核です。`info` から速度、横偏差、ヨーレート、スリップ、トラック幅などを取り出し、正負の各項を合算します。

正の項:

- `R_progress` : `ds * cos(heading)` を使う進行報酬
- `R_speed` : 前進速度に比例する速度ボーナス
- `R_centering` : センターライン近傍維持報酬
- `R_alive` : 微小な生存報酬
- `R_lap` : ラップ完了時のラップタイム依存ボーナス
- `R_previewSteer` : 先読み曲率と操舵方向の整合性報酬

負の項:

- `P_heading` : ヘディング誤差の二乗ペナルティ
- `P_boundary` : 境界接近時に指数的に増える罰則
- `P_slip` : 10 度超えの最大スリップ角に対する罰則
- `P_jerk` : 操舵変化量と大操舵に対する罰則
- `P_spin` : 大ヨーレートへの罰則
- `P_lowSpeedBrake` : 低速時にブレーキへ逃げる行動の罰則
- `P_terminal` : 終了理由に応じた固定罰則

先読み曲率では `rawObs(18:23)` の複数サンプルを重み付き合成し、現サーバーの操舵符号系に合わせて `-action(1)` 側で整合度を測っています。

### `checkTermination(obj, info, lapJustCompleted)`

エピソード終了判定を行います。

判定対象:

- `lap` : ラップ完了
- `offtrack` : 左右どちらかの余裕が `OffTrackMargin` 以下
- `spin` : 大ヨーレートかつ大ボディスリップ
- `reverse` : 後退速度
- `stall` : 一定時間ほとんど進めていない
- `timeout` : 最大ステップ超過
- `running` : 上記なし

`isDone` は `terminationReason ~= 'running'` で決まります。

### `[ds, lapJustCompleted] = computeProgressDelta(obj, s, vx)`

周回距離 `s` の差分を計算します。  
ラップ跨ぎでは `TrackLength` を使って差分を補正し、逆符号の大ジャンプを正常な 1 周繰り上がりとして扱います。

### `penalty = terminalPenalty(~, terminationReason)`

終了理由ごとの固定終端ペナルティを返します。

- `offtrack` : `-200`
- `spin` : `-150`
- `reverse` : `-100`
- `stall` : `-120`
- `timeout` : `-80`
- その他 : `0`

### `steerInput = decodeSteeringAction(obj, steerAction)`

正規化操舵 `[-1,1]` を実サーバー入力へ変換します。

- `CurrentVehicleParams.steering.torque_mode == true` なら `T_driver_max` を掛けて操舵トルク入力へ変換
- そうでなければ `max_angle` または `DeltaMax` を掛けて操舵角入力へ変換

つまり、このクラスは車両設定に応じてトルク操舵・角度操舵の両方を吸収できます。

### `obs = normalizeObservation(obj, raw)`

生観測 34 要素に `get_rl_vehicle_observation_vector(CurrentVehicleParams)` の 11 要素を結合し、`get_rl_observation_bounds()` の上下限で `[-1,1]` 正規化します。  
範囲外は最後に `max/min` でクリップされます。

### `recordRecentProgress(obj, ds)`

`CrawlWindowSteps` が有効な場合だけ、最近の `ds` を履歴に積みます。  
履歴長は `max(CrawlWindowSteps, StallGraceSteps)` に制限され、古い値は捨てられます。

### `[speedTarget, throttle, maxSteps] = getEffectiveRollingStart(obj)`

rolling start の有効値を返します。

- `RollingStartSpeed`、`RollingStartThrottle`、`RollingStartMaxSteps` を下限・上限で整える
- `RollingStartHoldEpisodes` まではフル有効
- その後 `RollingStartFadeEpisodes` にわたって線形に縮小
- 最終的に無効なら全値を 0 にする

初期学習では発進の難しさを和らげ、学習が進むと補助を消していく設計です。

### `[rawObs, currentS] = applyRollingStart(obj, rawObs, speedTarget, throttle, maxSteps)`

reset 直後にスロットルを自動印加し、最低速度まで立ち上げるヘルパーです。

ループ内では:

- `Client.step(0.0, throttle, 0.0)` で直進加速
- `currentS` を更新
- `vx >= speedTarget` なら終了
- オフトラック寸前なら安全のため終了

戻り値の `currentS` は、そのまま `PrevS` 初期値として使われます。

## 7. このクラスが前提にする周辺関数

- `resolve_rl_vehicle_params()` : 基準車両の構築
- `get_rl_observation_bounds()` : 観測上下限の共有契約
- `generateRandomVehicleParams()` : Domain Randomization 用サンプラ
- `get_rl_vehicle_observation_vector()` : 車両諸元 11 要素抽出
- `get_rl_server_host_port()` : 接続先ポート解決
- `FSAESimClient` : 実通信担当

## 8. 実装上の要点

- 行動空間は 2 次元、通信プロトコルは 3 次元というギャップをこのクラスが吸収しています。  
- 観測の最終次元は 45 で、旧エージェントとの互換性はありません。  
- 報酬、終了判定、randomization、rolling start がこのクラスに集中しており、RL 実験の性質はここで決まります。
