# `matlab/pou_hardpoint_design/tests/test_layer2.m`

## 1. 目的

Layer 2 と位置付けられている POU 最適化を最小構成で走らせる実行スクリプトです。FL のみ、マルチスタート 1 回、CSV のみ出力という軽量設定で `run_all_wheels` を呼びます。

## 2. 入出力と依存関係

- 形式: スクリプト
- 入力:
  - `../optimized_autocross.json`
  - スクリプト内で構築する `opts`
- 出力:
  - `module_dir/test_output` への CSV 出力
  - コンソール表示
- 依存関係:
  - `run_all_wheels`
  - `geometry`, `optimization`, `visualization`, `export` のパス追加

## 3. 全体処理フロー

1. 自身の場所から `module_dir` を求めます。
2. `run_all_wheels` が参照する各サブフォルダを `addpath` します。
3. `opts` を組み、`wheels=[0]`, `n_multistart=1`, `output_dir=test_output`, `export_csv=true`, `export_json=false`, `export_report=false`, `update_params=false`, `verbose=true` を設定します。
4. `optimized_autocross.json` を指定して `run_all_wheels` を実行します。
5. 実行前後に開始・終了メッセージを表示します。

## 4. ブロック詳細

- **パス設定**  
  このスクリプトは tests 配下にあるため、親ディレクトリを起点に相対パスを組んでいます。

- **オプション設定**  
  最適化そのものの検証に絞るため JSON 更新やレポート出力は無効化されています。

- **ローカル関数**  
  このファイルにローカル関数はありません。
