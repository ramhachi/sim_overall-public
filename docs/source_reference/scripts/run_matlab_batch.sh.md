# `scripts/run_matlab_batch.sh`

## 概要

プロジェクトルートと RL 用 path を自動設定したうえで `matlab -batch` を実行する薄い wrapper です。

## 入力

- 第1引数: 実行したい MATLAB コマンド列

## システム上の役割

CLI から RL / benchmark / utility を安全に呼ぶ共通土台です。
