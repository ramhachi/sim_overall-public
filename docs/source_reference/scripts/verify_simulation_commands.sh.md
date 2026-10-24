# scripts/verify_simulation_commands.sh

## 役割

MATLAB が組み立てる `laptime_sim` 実行コマンドを、Docker 経由で手元から再現確認するシェルです。

## 主な処理

- コンテナ内で `laptime_sim` をビルドする
- ASE Autocross の PID 条件を固定値で与える
- `manual_test.csv` を生成して末尾を確認する

## 使いどころ

- `run_full_pipeline.m` のコマンド生成が壊れていないか見たいとき
- Docker 側の実行経路だけを切り出して確認したいとき

## 注意点

- 出力ファイル名は手動確認向けの固定値です
- 常用ワークフローではなく、保守・切り分け用です
