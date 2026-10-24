# `matlab/dl_tire_model/removeMissingData.m`

## 1. 概要

特徴量行列と教師行列から NaN/Inf を含む行を除去し、除去統計も返す関数です。

## 2. ファイルの役割

学習前のデータ健全化を担当します。入力側 `X` と出力側 `Y` のどちらか一方でも不正値を含む行をまとめて除去し、除去件数と割合を `stats` に集約します。

## 3. インターフェース

- 形式: `[X_clean, Y_clean, stats] = removeMissingData(X, Y)`
- 引数:
  - `X`: `N×5` の入力特徴量行列
  - `Y`: `N×4` の出力教師行列
- 戻り値:
  - `X_clean`: 不正行除去後の入力
  - `Y_clean`: 不正行除去後の出力
  - `stats`:
    - `total_samples`
    - `removed_samples`
    - `clean_samples`
    - `removal_percentage`

## 4. 依存関係

- MATLAB 組み込み:
  - `isnan`
  - `isinf`
  - `any`
  - `sum`
  - `size`
  - `fprintf`

## 5. 処理の流れ

1. `isnan(X) | isinf(X)` を行方向にまとめ、入力側の不正行 `bad_rows_X` を作ります。
2. 同様に出力側の不正行 `bad_rows_Y` を作ります。
3. `bad_rows = bad_rows_X | bad_rows_Y` で最終的な除去対象を決めます。
4. 総サンプル数、除去数、残存数、除去率を `stats` に格納します。
5. 除去が 1 件以上あれば件数内訳を表示し、なければ全件正常であることを表示します。
6. `~bad_rows` で `X` と `Y` を同じ行インデックスで抽出します。

## 6. 関数・ローカル関数詳細

### `removeMissingData(X, Y)`

- 本ファイル唯一の公開関数です。
- 行の整合性を保つため、`X` と `Y` は同じ `bad_rows` で同時に削除されます。
- ログでは「特徴量側で不正な行数」と「教師側で不正な行数」を別々に表示します。

## 7. 注意点

- `X` と `Y` の行数一致確認は行っていません。
- `stats.removal_percentage` は `stats.total_samples` で除算しており、0 行入力への保護はありません。
- 欠損補完ではなく、該当行の完全削除です。
