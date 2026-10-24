# `matlab/rl_driver/loadRLAgentFromFile.m`

## 役割

`loadRLAgentFromFile` は、`agent.mat` や `AgentXX.mat` などの `.mat` ファイルから RL Agent を読み出し、**どの変数名から読み込んだか**まで返す loader です。  
checkpoint / final agent / deployment candidate を path から安全に読む共通入口として使われます。

---

## 処理の流れ

1. `resolveAgentPath` で実在するファイル path を解決する。
2. `load(resolvedPath)` で `.mat` 全体を struct として読む。
3. まず `agent` という変数名を探す。
4. 次に `saved_agent` を探す。
5. どちらも無ければ最初の変数を採用する。
6. Agent 本体、採用した変数名、解決済み path を返す。

---

## 関数詳細

## `loadRLAgentFromFile(agentPath)`

### 入力

- `agentPath`: 文字列または string。絶対 path / 相対 path のどちらでもよい

### 返り値

| 返り値 | 内容 |
| --- | --- |
| `agent` | `.mat` 内から取り出した Agent オブジェクト |
| `variableName` | どの変数名を採用したか |
| `resolvedPath` | 実在確認後の最終 path |

### 変数名の優先順位

1. `agent`
2. `saved_agent`
3. `fieldnames(data){1}`

多くの学習成果物では `agent.mat` に `agent` 変数が入る前提ですが、チェックポイントや外部保存物で変数名が違っても最低限読めるようにしています。

### エラー条件

- `resolveAgentPath` が実在ファイルを見つけられない
- `.mat` を load した結果に変数が一つも無い

後者は通常起こりにくいですが、壊れた `.mat` や想定外成果物を弾く役割があります。

---

## `resolveAgentPath(agentPath)`

path 解決 helper です。  
与えられた文字列がそのまま存在しなければ、関数自身の場所から順に候補を試します。

### 探索順

1. `agentPath` そのもの
2. `matlab/rl_driver/<agentPath>`
3. `matlab/<agentPath>`
4. `projectRoot/<agentPath>`

この順番により、呼び出し元の current folder に依存せず、リポジトリ相対指定をかなり柔軟に受け付けます。

### システム上の意味

- `trainPhase34Recovery` が再開元 Agent を読む
- `publishRLDeploymentCandidate` が path 指定された候補を読む
- 途中 checkpoint, final agent, deployment candidate を同じ API で扱える

つまりこの helper は、RL 運用系のファイル参照揺れを吸収する土台です。
