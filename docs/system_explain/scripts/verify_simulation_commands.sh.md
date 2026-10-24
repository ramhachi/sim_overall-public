# `scripts/verify_simulation_commands.sh`

## 1. 役割

Docker コンテナ内でラップタイムシミュレータをビルドし、**MATLAB から組み立てられる想定コマンドが実行できるか** を検証するスモークテスト用シェルです。

## 2. 入出力

- 実行例: `bash scripts/verify_simulation_commands.sh`
- 入力:
  - `docker compose` で起動する `fsae-sim` サービス
  - `/workspace/data/ASE_autocross_sim.csv`
- 出力:
  - ホスト側 `manual_test.csv`
  - 標準出力の成功/失敗メッセージ

## 3. トップレベルフロー

1. テスト用のトラック、半径、初速、PID ゲイン、最大スロットルを固定で定義します。
2. `docker compose run --rm fsae-sim` で `laptime_sim` をビルドします。
3. MATLAB 側が構成しそうなコマンド文字列 `CMD` を組み立てます。
4. 同コンテナでその `CMD` を実行し、生成 CSV を `/workspace` にコピーします。
5. ホストに `manual_test.csv` が現れたか確認し、最後の 5 行を表示します。

## 4. 重要な変数

- `TRACK_FILE`, `RADIUS`, `SIM_TIME`, `INIT_SPEED`, `OUTPUT_CSV`
- `KP_STEER`, `KD_STEER`, `KI_STEER`, `KH_STEER`, `KFF_STEER`
- `KP_SPEED`, `KI_SPEED`, `MAX_THROTTLE`

## 5. 注意点

- コメントでは「temp file」と書いていますが、実際は `build` 配下に一時生成した後、ホストへコピーします。
- `set -e` のため途中失敗時は即終了します。
