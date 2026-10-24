# `matlab/rl_driver/trainRLDriver.m`

## 役割

RL ドライバー学習の汎用エントリ関数です。  
個別スクリプト群が固定のトラック構成や報酬設計を持つのに対し、この関数は **トラック名・車両パラメータ・ドメインランダム化・保存条件・再開学習・環境上書き**を可変にして、幅広い学習 run を 1 本で扱えるようにしています。

## 関数シグネチャ

```matlab
trainRLDriver(trackName, vehicleParams, varargin)
```

### 必須・準必須引数

- `trackName`
  - 対象トラック名
  - 省略時は `'circle'`
- `vehicleParams`
  - 車両パラメータ構造体
  - 省略時は空構造体 `struct()`

### 主な名前付き引数

- `MaxEpisodes`
- `StopReward`
- `SaveDir`
- `UseParallel`
- `DomainRand`
- `RandInterval`
- `RandStrength`
- `ResumeAgent`
- `EnvOverrides`
- `MaxStepsPerEpisode`
- `ScoreAveragingWindowLength`
- `SaveAgentCriteria`
- `SaveAgentValue`
- `Plots`
- `Verbose`

## この関数の実行フロー

1. 引数の既定値を補う。
2. `inputParser` で学習 run の設定を読み取る。
3. 保存ディレクトリを作る。
4. `FSAEEnvironment(trackName, vehicleParams)` を構築する。
5. 必要に応じてエピソード長、環境上書き、ドメインランダム化を適用する。
6. 新規エージェント作成または ResumeAgent による再開を行う。
7. `rlTrainingOptions` を構成する。
8. `train` で学習する。
9. エージェントと学習統計を保存する。

## セクション詳細

### 関数ヘッダ

```matlab
function trainRLDriver(trackName, vehicleParams, varargin)
```

- 学習実行を外部から再利用しやすくするため、関数化されています。
- 先頭で
  ```matlab
  if nargin < 1, trackName = 'circle'; end
  if nargin < 2, vehicleParams = struct(); end
  ```
  としており、最小呼び出しは `trainRLDriver()` でも成立します。

### `%% オプション解析`

`inputParser` で多数のオプションを受け付けます。  
各項目の意味は次の通りです。

- `MaxEpisodes = 1000000`  
  非常に長い学習 run も許容する既定値です。
- `StopReward = 2000`  
  停止基準となる平均報酬です。
- `SaveDir = fullfile('rl_results', trackName)`  
  トラック名ごとに保存先を分けます。
- `UseParallel = false`  
  Reinforcement Learning Toolbox 側の並列学習フラグです。
- `DomainRand = false`  
  ドメインランダム化有効化フラグです。
- `RandInterval = 100`
- `RandStrength = 0.3`
- `ResumeAgent = []`
- `EnvOverrides = struct()`  
  環境プロパティの追加上書き用です。
- `MaxStepsPerEpisode = []`
- `ScoreAveragingWindowLength = 100`
- `SaveAgentCriteria = 'EpisodeReward'`
- `SaveAgentValue = []`  
  未指定時は条件に応じて自動補完されます。
- `Plots = 'training-progress'`
- `Verbose = true`

この section は、個別スクリプトで固定値だったものを引数化している部分です。

### `%% 出力ディレクトリ作成`

```matlab
if ~exist(opts.SaveDir, 'dir')
    mkdir(opts.SaveDir);
end
```

- 学習成果物の保存先を保証します。
- `trackName` ベースの既定保存先を使うため、トラックごとに整理しやすい構造です。

### `%% サーバー起動確認`

この section 名ですが、実際にはサーバー生死チェック処理は書かれていません。  
標準出力へ次を表示するだけです。

- `Track`
- `Max Episodes`
- `Save Directory`

つまり、ここは「起動確認」というより run 情報の表示 section です。

### `%% 環境構築`

```matlab
env = FSAEEnvironment(trackName, vehicleParams);
```

- 指定トラックと指定車両条件で環境を作ります。
- `vehicleParams` を直接渡せるため、最適化済み車両やドメインランダム化前の基準車両を明示できます。

続く処理の意味:

1. `MaxStepsPerEpisode` が指定されていれば `env.MaxEpisodeSteps` を上書き
2. `applyRLEnvOverrides(env, opts.EnvOverrides, 'Env override')` で任意の環境パラメータを一括上書き
3. `opts.DomainRand` が真なら
   - `DomainRandEnabled = true`
   - `DomainRandInterval = opts.RandInterval`
   - `DomainRandStrength = opts.RandStrength`
   - `DomainRandBounds = getVehicleParamBounds()`
   を設定

この section があるため、単なる固定トラック学習ではなく、幅広い実験条件に対応できます。

### `%% エージェント構築`

```matlab
if ~isempty(opts.ResumeAgent)
    agent = opts.ResumeAgent;
    assertAgentObservationDimension(...);
else
    agent = createSACAgent(env);
end
```

- `ResumeAgent` がある場合は継続学習です。
- 観測次元の整合性を `assertAgentObservationDimension` で確認し、古い agent を別環境へ誤投入する事故を防ぎます。
- 新規学習時は `createSACAgent(env)` により環境仕様へ合わせた agent を作ります。

### `%% 学習オプション`

```matlab
trainOpts = rlTrainingOptions;
```

この section はオプションの集約点です。

- `MaxEpisodes`
- `MaxStepsPerEpisode`
- `ScoreAveragingWindowLength`
- `StopTrainingCriteria = 'AverageReward'`
- `StopTrainingValue = opts.StopReward`
- `SaveAgentCriteria = opts.SaveAgentCriteria`
- `SaveAgentDirectory = opts.SaveDir`
- `Plots = opts.Plots`
- `Verbose = opts.Verbose`

#### `SaveAgentValue` 自動補完ロジック

`SaveAgentValue` が未指定の場合:

- `SaveAgentCriteria == 'EpisodeReward'`
  - `opts.StopReward * 0.8`
- `SaveAgentCriteria == 'EpisodeCount'`
  - `max(1, floor(opts.MaxEpisodes / 10))`
- それ以外
  - `error(...)`

つまり、保存条件を切り替える際に閾値を忘れても、主要ケースでは自動で妥当値を入れる設計です。

#### 並列設定

```matlab
if isprop(trainOpts, 'UseParallel')
    trainOpts.UseParallel = opts.UseParallel;
end
```

- Toolbox やバージョン差異に配慮し、`UseParallel` プロパティが存在する場合のみ設定しています。

### `%% 学習実行`

```matlab
trainingStats = train(agent, env, trainOpts);
```

- ここで実際の RL 学習を開始します。
- 個別スクリプトと異なり、ここでは報酬重みや TrackPool 自体は直接いじらず、環境とオプションで制御します。

### `%% 結果保存`

```matlab
save(fullfile(opts.SaveDir, 'trained_agent.mat'), 'agent');
save(fullfile(opts.SaveDir, 'training_stats.mat'), 'trainingStats');
```

- 学習済みエージェント
- 学習統計

を保存します。  
最後に保存先を表示して終了します。

## この関数の価値

- 個別学習スクリプトの共通基盤
- quickstart から直接呼べる汎用入口
- ドメインランダム化、継続学習、保存条件制御など実験向けの柔軟性が高い

## 注意点

- section 名に対して、実際の「サーバー起動確認」は行っていません。
- 環境オブジェクトの `delete` はこの関数内にありません。環境解放は `train` や MATLAB の寿命管理に委ねる設計です。
