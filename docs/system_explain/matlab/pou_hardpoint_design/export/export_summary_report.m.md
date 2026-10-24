# `matlab/pou_hardpoint_design/export/export_summary_report.m`

## 1. 目的

POU 最適化結果の人間可読レポートをテキストで生成する関数です。収束状況、座標、性能指標、パッケージング、前提条件を 1 ファイルにまとめます。

## 2. 入出力と依存関係

- 入力:
  - `results`
  - `geom`
  - `output_dir`（省略時 `.`）
- 出力:
  - `report_path`: 生成した `pou_report_<wheel>.txt`
- 依存関係:
  - `datestr(now, ...)`
  - `rad2deg`
  - `mkdir`, `fopen`, `fprintf`, `fclose`

## 3. 全体処理フロー

1. 出力ディレクトリを準備します。
2. `geom.wheel_index` からホイール名を決め、レポートファイル名を作ります。
3. ヘッダにホイール名と生成日時を書きます。
4. `optimization_exitflag` で `CONVERGED / PARTIAL / FAILED` を切り替えます。
5. POU 点と A アーム・タイロッド・キングピン座標を順に出力します。
6. 誤差、ロッド角、IR、pushrod 長を閾値付きで評価します。
7. パッケージング違反があれば全文列挙します。
8. rim 半径や clearance などの前提条件を書き、最後に「A-arm hardpoints are PLACEHOLDER values.」という注意書きを入れます。

## 4. 関数詳細

### `export_summary_report(results, geom, output_dir)`

- **状態判定**  
  `exitflag >= 1` を収束、`== 0` を反復上限による部分解、それ未満を失敗として表示します。

- **性能閾値**  
  RMS 誤差は 2 mm 未満、ロッド角は `geom.rod_end_angle_max` 未満、IR は 0.05～0.50 を通過条件として文字列表示します。

- **前提条件の明示**  
  幾何拘束に使った `rim_radius`, `rim_clearance_min`, `pushrod_length_min/max`, `rod_end_angle_max` をレポートへ残します。

- **末尾注意書き**  
  実装上は `define_suspension_geometry.m` の値が仮置きである可能性を前提に、製造前に CAD 実値へ更新するよう警告しています。

- **ローカル関数**  
  このファイルにローカル関数はありません。
