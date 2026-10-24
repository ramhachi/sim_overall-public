# `cpp/src/main_rl_actor_check.cpp`

## 1. ファイルの目的
MATLAB から書き出した RL アクタの期待出力と、C++ `RLDriver` 推論結果の一致性を CSV ベースで検証するチェックツールです。

## 2. 実行上の役割
- 学習済みアクタの C++ 移植確認を行います。
- 量子化差・重み読込ミス・前処理差分を早期発見するための回帰テスト入口です。

## 3. 依存関係
- `fsae/driver/rl_driver.hpp`
- `<fstream>`, `<sstream>`, `<vector>`, `<algorithm>`, `<cmath>`

## 4. 入力/出力
- CLI:
  - `--actor-dir <dir>` 必須
  - `--test-data <csv>` 任意（既定: `<actor-dir>/rl_actor_test_data.csv`）
  - `--threshold <value>` 任意（既定: `1e-3`）
- 出力:
  - 標準出力に行数、各 action 次元の最大絶対誤差、全体最大誤差
  - しきい値超過時は終了コード `2`

## 5. 関数・メソッド・クラスの網羅的な説明

### `print_usage()`
- 使用方法を標準エラーへ表示します。

### `split_csv_to_doubles(const std::string& line, std::vector<double>& out)`
- 1 行の CSV 文字列を `double` 配列へ変換します。
- 変換失敗時は `false` を返します。

### `main(int argc, char** argv)`
- 引数を解析し、`RLDriver` にアクタを読み込みます。
- テスト CSV の各行を
  - 先頭 `obs_dim` 列: 観測
  - 続く `act_dim` 列: 期待 action
  として解釈します。
- `driver.infer_normalized_action(obs)` の出力と期待値の最大絶対誤差を各次元で追跡します。
- 全体最大誤差が `threshold` を超えると失敗終了します。

