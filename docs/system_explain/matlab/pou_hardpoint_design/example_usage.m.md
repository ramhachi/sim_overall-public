# `matlab/pou_hardpoint_design/example_usage.m`

## 1. 目的

POU ハードポイント逆運動学ワークフローの英語版デモです。`optimized_autocross.json` を読み込み、前輪 1 輪分の最適化を実行し、結果確認とプロット生成までを一通り示します。

## 2. 入出力と依存関係

- 形式: スクリプト
- 主入力:
  - `../optimized_autocross.json`
  - スクリプト内で組み立てる `options` 構造体
- 主出力:
  - 変数 `results`
  - 標準出力への進捗・評価表示
  - `plot_comparison(results)` の図
- 依存関係:
  - `addpath('geometry')`
  - `addpath('optimization')`
  - `addpath('visualization')`
  - `main_pou_hardpoint_optimizer`
  - `plot_comparison`

## 3. 全体処理フロー

1. ワークスペースを初期化し、必要フォルダを `addpath` します。
2. `params_file = '../optimized_autocross.json'` を設定し、存在しなければ `error` で停止します。
3. `options` に `wheel_index=0`、`n_multistart=5`、操舵角範囲 `[-30, 30]` deg、点数 `61`、`verbose=true` を設定します。
4. `main_pou_hardpoint_optimizer(params_file, options)` を実行し、`tic/toc` で総時間を計測します。
5. `results` から M ピボット、ベルクランク、RMS 誤差、ジャッキアップ誤差、キャンバー誤差、ロッド角、IR、パッケージング判定を表示します。
6. `plot_comparison(results)` を `try/catch` で呼び、失敗時は例外メッセージだけを出します。
7. SolidWorks へのエクスポートは未実装として案内文のみ表示します。

## 4. スクリプト内ブロック詳細

- **初期化ブロック**  
  `clear; clc; close all;` で前回状態を消し、再現しやすい実行条件にします。

- **Step 1: パラメータ読込**  
  JSON への相対パスを固定し、ファイルがなければ即停止します。ここでは JSON 内容の妥当性検証までは行わず、存在確認のみです。

- **Step 2: オプション構築**  
  `wheel_index=0` なので対象は FL です。`['FL', 'FR', 'RL', 'RR'](options.wheel_index*2 + (1:2))` という 2 文字切り出しで表示用ホイール名を作っています。

- **Step 3: 最適化実行**  
  実処理はすべて `main_pou_hardpoint_optimizer` に委譲され、このスクリプトは実行例の薄いラッパです。

- **Step 4: 結果解析**  
  `optimization_exitflag`、座標、誤差、ロッドエンド角、インストレーション比を閾値付きで表示します。`results.violations` がある場合は全件列挙します。

- **Step 5: 可視化**  
  `plot_comparison` の返す figure を受けますが、保存はコメントアウトされた例のみで、実際には保存しません。

- **Step 6: 将来機能の案内**  
  `export_to_solidworks(...)` は説明文として出すだけで、関数呼び出しは行いません。

- **ローカル関数**  
  このファイルにローカル関数はありません。
