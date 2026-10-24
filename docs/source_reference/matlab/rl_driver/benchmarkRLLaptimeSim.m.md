# `matlab/rl_driver/benchmarkRLLaptimeSim.m`

## 概要

RL actor export を使って `laptime_sim` を走らせ、完走可否・ラップタイム・wall-clock を計測するベンチマーク関数です。標準では `autocross` と `skidpad` を対象にします。

## 主な入力

- `Tracks`
- `ActorDir`
- `ProjectRoot`
- `Rebuild`
- `VehicleParams`
- `SaveJson`

## 主な出力

- `results(track, lap_time, lap_completed, wall_clock_sec, actor_dir, benchmark_mode, benchmark_command, ...)`

## システム上の役割

RL 学習済みドライバーの「速さ」と「計算コスト」を同じ入口で測る運用評価関数です。
