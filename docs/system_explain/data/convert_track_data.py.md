# `data/convert_track_data.py`

## 役割

- `ASE_autocross.csv` をシミュレーション用の `x, y, width` 形式へ変換し、**閉ループ前提の平滑化**まで行う前処理スクリプトです。
- 元 CSV の `x` 列と `z` 列を取り出し、`z -> y` へ軸変換して `ASE_autocross_sim.csv` を生成します。

## 運用上の入口

### `convert_track()`
- スクリプト実行時の標準入口です。
- 入力・出力ファイルは `input_file` / `output_file` の絶対パスで固定されています。

## 主要関数

### `smooth_points(points, window_size=5)`
- **入力**: `(x, y)` タプル列、移動平均窓幅
- **出力**: 平滑化済み `(x, y)` タプル列
- **処理**: `% n` を使う循環インデックスで、トラックを閉ループとして平滑化します。
- **注意点**: 点数が窓幅未満なら元データをそのまま返します。

### `convert_track()`
- **入力**: 元 CSV ファイル
- **出力**: `x, y, width` ヘッダ付き CSV
- **副作用**: `data/ASE_autocross_sim.csv` を上書き生成
- **主な検証**:
  - ファイル存在確認
  - 空ファイル検出
  - `x` / `z` 列の存在確認
  - 数値変換できない行のスキップ

## データフロー

1. `ASE_autocross.csv` を開く
2. ヘッダから `x` と `z` 列位置を特定
3. 各行を `(x, z)` から `(x, y)` へ変換
4. `smooth_points(..., window_size=5)` で閉ループ平滑化
5. 幅 `4.5` m を付与して `ASE_autocross_sim.csv` に書き出す

## 入出力・副作用

- **入力**: `data/ASE_autocross.csv`
- **出力**: `data/ASE_autocross_sim.csv`
- **標準出力**: 読み込み状況、エラー、変換成功件数
- **副作用**: 出力 CSV の新規作成または上書き

## システム内での位置づけ

- Assetto Corsa 由来や手加工トラックを、シミュレータが扱う簡易 CSV へ寄せる**トラック変換の入口**です。
- 後続の `analyze_track_quality.py` がこの出力を点検するため、両者で前処理→品質確認の流れを構成します。
