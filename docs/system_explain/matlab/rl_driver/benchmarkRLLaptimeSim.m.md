# `matlab/rl_driver/benchmarkRLLaptimeSim.m`

## 役割

RL actor export を使って `laptime_sim` を実行し、トラックごとの完走可否・ラップタイム・壁時計時間を計測する benchmark 関数です。デフォルトでは `autocross` と `skidpad` を測ります。

## 入力

- `Tracks`
- `ActorDir`
- `ProjectRoot`
- `Rebuild`
- `VehicleParams`
- `SaveJson`

## 全体フロー

1. `ensure_rl_driver_paths()` で bootstrap。
2. `resolveProjectRoot()` と `resolveBenchmarkActorDir()` で実行基準を解決。
3. `resolve_rl_vehicle_params()` で benchmark 用 params を得る。
4. 各 track ごとに `writeBenchmarkParamsFile()` で一時 JSON を書く。
5. `runBenchmarkCommand()` で Docker `exec` を優先し、失敗したら host binary を試す。
6. コマンド出力から JSON payload を抜き出して `results(i)` に詰める。
7. benchmark 結果を `output/runtime/rl_laptime_benchmark_*.json` へ保存。

## 重要点

- `--params_file` を明示して RL steering contract と車両 params の不整合を避けます。
- `benchmark_mode` と `benchmark_command` を保存するので、どの経路で測ったかを後で追跡できます。
- 一時 params JSON は track ごとに作成し、実行後に削除します。

## システム上の位置づけ

RL の「速いか」「重いか」を同時に測る標準ベンチです。EasyStart の RL メニューからも到達します。
