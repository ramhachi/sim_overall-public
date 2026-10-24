# `scripts/run_matlab_batch.sh`

## 役割

CLI から MATLAB batch 実行を呼ぶ共通 wrapper です。project root へ移動し、`matlab/` と `matlab/rl_driver/` を path へ足した状態で引数コマンドを実行します。

## 処理の流れ

1. `SCRIPT_DIR` と `PROJECT_ROOT` を解決。
2. `matlab` 実行ファイル存在確認。
3. 第 1 引数が無ければ usage を表示して終了。
4. `matlab -batch "cd(...); addpath(...); <MATLAB_COMMANDS>"` を実行。

## システム上の位置づけ

RL 追加学習や benchmark を shell から再現するための共通土台です。
