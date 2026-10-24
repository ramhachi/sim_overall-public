# `matlab/pou_hardpoint_design/test_json_loading.m`

## 1. 目的

`optimized_autocross.json` を POU 系コードが期待する形式で読めるか確認するスクリプトです。存在確認、読込、構造確認、POU 多項式抽出、車両寸法抽出までを段階的に試します。

## 2. 入出力と依存関係

- 形式: スクリプト
- 主入力: `../optimized_autocross.json`
- 主出力: 読込結果と抽出値のコンソール表示
- 依存関係:
  - `load_vehicle_artifact`
  - 親ディレクトリ追加用 `addpath(fileparts(this_dir))`

## 3. 全体処理フロー

1. 自身の親ディレクトリを `addpath` して helper 関数を見えるようにします。
2. `json_file` の存在確認を行い、無ければその場で終了します。
3. `load_vehicle_artifact` で JSON を読み、失敗したら例外内容を表示して終了します。
4. `json_data.params.suspension.pou_jack_c1` まで存在するかを順に確認します。
5. `pou_jack_c1/c2`, `pou_camber_c1/c2` を 4 輪分表示します。
6. `params.mass` から `wheelbase`, `track_f`, `track_r` を表示します。
7. FL 用に `target_poly` を組み立て、次に呼ぶべき `main_pou_hardpoint_optimizer` の例を表示します。

## 4. ブロック詳細

- **構造確認部**  
  `params`、`params.suspension`、`params.suspension.pou_jack_c1` の順に検査しています。いずれかが無ければ途中 `return` します。

- **POU 抽出部**  
  4 要素配列をそのまま `fprintf` に渡して輪別の係数を可視化します。

- **FL 目標多項式生成部**  
  `idx = 1` を使い、MATLAB の 1 始まり前提で FL を選択します。

- **ローカル関数**  
  このファイルにローカル関数はありません。
