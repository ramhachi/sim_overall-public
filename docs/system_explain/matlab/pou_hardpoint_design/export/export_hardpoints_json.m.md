# `matlab/pou_hardpoint_design/export/export_hardpoints_json.m`

## 1. 目的

POU 最適化結果を JSON 化する関数です。新規作成にも既存 JSON への追記にも対応し、`pou_hardpoints.<wheel>` の下へ輪別データを保存します。

## 2. 入出力と依存関係

- 入力:
  - `results`
  - `geom`
  - `output_path`（省略時 `pou_hardpoints.json`）
- 出力:
  - `json_path`: 書き込んだ JSON パス
- 依存関係:
  - `jsonencode`, `jsondecode`, `fileread`
  - `fopen`, `fwrite`, `fclose`

## 3. 全体処理フロー

1. ホイール名 `FL/FR/RL/RR` を `geom.wheel_index` から決めます。
2. `results` と `geom` から `wheel_data` を組み立て、各値を `round` で丸めます。
3. 既存 JSON があれば読み込み、無ければ空構造体から開始します。
4. `data.pou_hardpoints` が無ければ新設します。
5. `data.pou_hardpoints.(wn)` を今回輪の `wheel_data` で更新します。
6. `jsonencode` 後に単純な `strrep` で改行を入れ、ファイルへ保存します。

## 4. 関数詳細

### `export_hardpoints_json(results, geom, output_path)`

- **保存項目**  
  POU の最適化結果だけでなく、上下 A アーム、タイロッド、キングピン情報まで輪別 JSON に含めます。

- **丸め精度**  
  座標の多くは小数 6 桁、pushrod 長と IR は 4 桁、ロッド角は 2 桁、誤差は 3 桁です。

- **pretty-print の実態**  
  コメントでは整形出力を意図していますが、実装は `strrep` で `{`, `}`, `,"` を置換する簡易整形です。ネストが深い JSON を厳密に整える実装ではありません。

- **ローカル関数**  
  このファイルにローカル関数はありません。
