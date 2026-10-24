# `scripts/analysis/plot_velocity_profile.py`

## 1. 役割

ラップタイムや経路追従の CSV から、**速度プロファイル・曲率・導出加速度** をまとめて確認する解析スクリプトです。

## 2. 入出力

- 実行例: `python3 scripts/analysis/plot_velocity_profile.py <csv_file> <output_png>`
- 入力列: `time`, `s`, `speed`, `target_speed`, `kappa`
- 出力: 3 段プロット PNG

## 3. 関数

### `plot_velocity_profile(csv_file, output_file)`
- 役割: CSV を読み、3 つのサブプロットを作成します。
- 処理内容:
  1. `csv.DictReader` で必要列だけを float 化して蓄積
  2. `s` に対する実速度/目標速度
  3. `s` に対する曲率
  4. 時間差分から縦加速度、`v^2*kappa` から横加速度を導出
- 副作用: PNG 保存、保存先を print
- 失敗時: ファイル未発見や空データならメッセージを出して return

## 4. CLI 挙動

- 引数不足時は usage を出し、終了コード 1 で終了します。
