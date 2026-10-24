# `matlab/rl_driver/get_rl_server_host_port.m`

## 1. 役割

このファイルは、MATLAB 側が RL サーバーへ接続するときに使うホスト側ポート番号を決める小さなユーティリティです。  
既定値は `22345` ですが、環境変数 `FSAE_HOST_RL_PORT` があればそれを優先します。

## 2. 関数一覧

1. `get_rl_server_host_port()`
2. `parse_port_env(env_name, default_port)`

## 3. `get_rl_server_host_port()` 詳細

呼び出し側に公開される入口関数です。  
実際の処理は `parse_port_env('FSAE_HOST_RL_PORT', 22345)` を呼ぶだけで、環境変数未設定時は `22345` を返します。

主な利用先:

- `FSAESimClient` コンストラクタ
- `FSAEEnvironment` コンストラクタ
- `startRLServer`

これにより、ポート番号の決定ロジックを各所で重複させずに済んでいます。

## 4. `parse_port_env(env_name, default_port)` 詳細

環境変数からポート番号を安全に取り出す内部ヘルパーです。

手順:

1. `getenv(env_name)` で文字列取得
2. `strtrim` で前後空白除去
3. 空なら `default_port` を返す
4. `str2double` で数値化
5. 有限値・正数・整数かを確認
6. 条件を満たさなければ `default_port`
7. 妥当ならその値を返す

つまり「設定されていれば使うが、不正値なら黙って既定値へ戻す」方針です。

## 5. 実装上の要点

- 例外を投げずフォールバックするため、環境差分のある実験環境でも壊れにくいです。  
- ポート設定ロジックが一箇所に集約されているため、将来の変更点もここに限定できます。
