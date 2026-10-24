# `python/scripts/compare_matlab_cpp.py`

## 1. 役割

Quarter Car の MATLAB 結果と C++ 結果を **CSV 同士で突き合わせる比較検証スクリプト** です。  
誤差統計を計算し、比較図とヒステリシス差分を可視化して、簡易 PASS/FAIL を返します。

## 2. 実行と入出力

- 実行例: `python3 python/scripts/compare_matlab_cpp.py <cpp.csv> <matlab.csv> [output_dir]`
- 入力: `time`, `z_s`, `z_u`, `dz_s`, `dz_u`, `F_spring`, `F_damper`, `F_tire` を持つ 2 つの CSV
- 出力:
  - 標準出力の誤差表
  - `verification_comparison.png`
  - 終了コード `0`(合格) / `1`(不合格)

## 3. 主要制御フロー

1. `main()` が引数から C++/MATLAB CSV と出力先を決めます。
2. `load_results()` で 2 つの CSV を DataFrame として読みます。
3. `compute_errors()` が共通長さ `n` までで誤差配列と RMSE などを計算します。
4. `print_error_summary()` が変数ごとの閾値判定を表示します。
5. `plot_comparison()` が比較図を保存・表示します。
6. 合否に応じてプロセス終了コードを返します。

## 4. 関数

### `load_results(cpp_path: str, matlab_path: str)`
- 入力: 2 つの CSV パス
- 戻り値: `(cpp_df, matlab_df)`
- 副作用: 読み込んだ点数を標準出力へ表示します。

### `compute_errors(cpp_df, matlab_df)`
- 役割: 共通列について差分配列と統計量を計算します。
- 戻り値:
  - `errors`: 列ごとの `max_abs`, `rmse`, `mean`, `diff`
  - `n`: 比較に使ったサンプル数
- 注意: 長さ不一致時は短い方に切り詰めます。

### `print_error_summary(errors: dict)`
- 役割: 変数ごとの誤差一覧と合否を出力します。
- 判定:
  - 力列 (`F_`) は 1.0
  - それ以外は 0.01
- 戻り値: 全項目が閾値内なら `True`

### `plot_comparison(cpp_df, matlab_df, errors, n, output_dir)`
- 役割: 6 枚の比較図を作成します。
- 内容:
  - `z_s`, `z_u`, `F_damper` の重ね描き
  - `z_s`, `F_damper` 誤差
  - ダンパ F-V ループ比較
- 副作用:
  - 出力ディレクトリ生成
  - PNG 保存
  - `plt.show()` 実行

### `main()`
- 役割: CLI 入口です。
- 副作用: デフォルトパス `/workspace/...` を使うため、コンテナ実行前提の既定値を持ちます。
