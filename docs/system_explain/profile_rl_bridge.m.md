# `profile_rl_bridge.m`

## 役割

`profile_rl_bridge.m` は、**MATLAB と RL サーバーの橋渡し層の実行性能を簡易計測するためのスクリプト** です。対象は次の 2 系統です。

1. `FSAESimClient` を使った TCP クライアント直接呼び出し
2. `FSAEEnvironment` を使った RL 環境ラッパ呼び出し

目的は、`reset()` と `step()` の平均所要時間を比較し、1 エピソードあたりの概算実行時間を把握することです。

---

## スクリプトとしての位置づけ

- 関数ではなくトップレベルスクリプトです。
- 実行すると、その場で RL サーバー接続・環境生成・反復計測を行います。
- ベンチマーク対象トラックは固定で `circle` です。
- `try/catch` やクリーンアップガードは無いため、前提環境が整っている状態での開発者向け計測スクリプトと考えるのが適切です。

---

## 実行前提

- `matlab/rl_driver` 配下のクラス群が利用可能であること
- RL サーバーが接続可能であること
- `get_rl_server_host_port()` が有効なポートを返すこと
- `FSAESimClient` と `FSAEEnvironment` がパス上で解決できること

---

## 操作フロー

1. `addpath('matlab/rl_driver')` で RL ドライバー関連コードをパス追加。
2. TCP クライアントの性能計測を開始。
3. `FSAESimClient('localhost', get_rl_server_host_port())` を生成。
4. `connect()` → `init('circle')` を実行。
5. `reset()` を 100 回回して平均時間を計測。
6. `step(0, 0.5, 0)` を 1000 回回して平均時間を計測。
7. `disconnect()` で TCP 接続を閉じる。
8. RL 環境ラッパ `FSAEEnvironment('circle')` を生成。
9. `reset(env)` を 100 回計測。
10. `step(env, [0; 0.5; 0])` を 1000 回計測。
11. `reset + 3000 * step` で 1 エピソード概算時間を出力。

---

## スクリプト内の見える処理ブロック

### 1. パス設定
```matlab
addpath('matlab/rl_driver');
```
- RL 関連クラスや補助関数を見えるようにします。
- 相対パス指定のため、通常はプロジェクトルート起点での実行を想定しています。

### 2. TCP Client Profiling
```matlab
fprintf('=== Profiling TCP Client ===\n');
client = FSAESimClient('localhost', get_rl_server_host_port());
client.connect();
client.init('circle');
```
- RL サーバーへの生 TCP 経路を直接計測する準備です。
- `init('circle')` でシナリオ初期化を行い、その後の `reset` / `step` 計測条件を固定します。

### 3. Reset performance
```matlab
N_RESET = 100;
tic;
for i = 1:N_RESET
    client.reset();
end
t_reset = toc / N_RESET;
```
- TCP クライアント経由の `reset()` 平均時間を測ります。
- `tic/toc` の総時間を試行回数で割る単純平均方式です。
- ウォームアップ専用ステップは無く、100 回全体をそのまま平均します。

### 4. Step performance
```matlab
N_STEP = 1000;
client.reset();
tic;
for i = 1:N_STEP
    client.step(0, 0.5, 0);
end
t_step = toc / N_STEP;
```
- TCP クライアント経由の `step()` 平均時間を計測します。
- 入力は固定で、
  - steering: `0`
  - throttle: `0.5`
  - brake: `0`
 です。
- `client.reset()` を 1 回挟み、初期条件を揃えてから反復します。

### 5. TCP client disconnect
```matlab
client.disconnect();
```
- TCP 計測終了後に接続を閉じます。
- 例外時保護は無いので、途中失敗時は切断まで到達しない可能性があります。

### 6. RL Environment Profiling
```matlab
fprintf('\n=== Profiling RL Environment ===\n');
env = FSAEEnvironment('circle');
```
- MATLAB 側 RL 環境オブジェクト全体のラップコストを測る準備です。
- TCP クライアントの純粋通信寄りコストに対し、こちらは環境 API 層のラッピングコストも含みます。

### 7. Env Reset
```matlab
tic;
for i = 1:N_RESET
    reset(env);
end
t_env_reset = toc / N_RESET;
```
- `FSAEEnvironment` の `reset` 平均時間を計測します。
- RL Toolbox 風のインターフェースである `reset(env)` 呼び出しを直接評価します。

### 8. Env Step
```matlab
reset(env);
tic;
for i = 1:N_STEP
    step(env, [0; 0.5; 0]);
end
t_env_step = toc / N_STEP;
```
- `step(env, action)` の平均時間を測ります。
- 行動ベクトルは列ベクトル `[steer; throttle; brake] = [0; 0.5; 0]` です。
- TCP クライアント版と入力条件を揃えて比較できるようにしています。

### 9. 1 エピソード推定時間
```matlab
est_ep_time = t_env_reset + (t_env_step * 3000);
```
- 1 回の reset と 3000 ステップで 1 エピソードを概算します。
- 厳密なプロファイラではなく、学習試行時間の大まかな目安を素早く把握するための計算です。

---

## 出力される指標

- `TCP Client Avg Reset`
- `TCP Client Avg Step`
- `RL Env Avg Reset`
- `RL Env Avg Step`
- `Estimated Time for 1 Episode (3000 steps)`

単位は reset / step がミリ秒、エピソード時間が秒です。

---

## このスクリプトで分かること

1. RL サーバーとの生通信コスト
2. MATLAB RL 環境ラッパ込みのステップコスト
3. 学習 1 エピソードあたりの概算時間
4. 通信ボトルネックか、環境ラップのオーバーヘッドかの粗い切り分け

---

## 注意点

- トラックは `circle` 固定です。トラック依存差分はこのスクリプト単体では見えません。
- 入力も固定なので、行動多様性による性能差は測っていません。
- `try/catch` が無いため、接続失敗時の後始末は自動化されていません。
- 初回接続や初回 JIT コンパイルの影響を分離していないため、厳密ベンチマークではなく簡易プロファイル用途です。

---

## 読みどころ

このスクリプトの本質は、**「RL サーバーへ直接投げる場合」と「MATLAB RL 環境として包んだ場合」の差を、同一条件でざっくり比較すること** にあります。運用時の学習時間見積りや、ブリッジ層最適化の優先順位判断に向いています。
