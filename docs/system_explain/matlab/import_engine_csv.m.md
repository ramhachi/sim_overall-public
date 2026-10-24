# `matlab/import_engine_csv.m`

## 1. ファイルの目的

実測らしき CSV `test_04.CSV` を解析し、FSAE 用エンジントルク曲線を MATLAB 変数と `.mat` に変換する検討用スクリプトです。

## 2. 主な入出力または依存関係

- 入力: `test_04.CSV`
- 出力:
  - 標準出力への `rpm_data` / `torque_data` 片
  - `custom_engine_curve.mat`
  - `engine_data` 構造体
- 依存関係:
  - `detectImportOptions`
  - `readmatrix`
  - `save`

## 3. 処理の流れ / アルゴリズム

1. CSV を読み込み、列の意味をコメント内で検証します。
2. 2 列目を torque[Kgm] と仮定し、`9.80665` 倍して Nm へ変換します。
3. 妥当な RPM とトルクだけ抽出してソートします。
4. 低回転側は `[1000, 3000, 5000]` に仮定値を追加し、高回転側は必要に応じて 14000rpm / 30Nm を補います。
5. 重複 RPM を除去し、`ConceptTradeOffs.m` へ貼れる形式で配列を表示します。
6. `engine_data` を `.mat` 保存します。

## 4. ファイル内の定義

### スクリプト本体

- コメントが長く、CSV 列の解釈と単位推定そのものが主目的です。
- 実データ解析より「どの列をトルクと見なすべきか」の考察を記録し、その結論に基づいて簡易曲線を生成します。
- `save('custom_engine_curve.mat', 'engine_data')` は途中に 1 回、末尾にもう 1 回現れますが、有効な保存は後半で `engine_data` 生成後に行われます。

## 5. 注意点・前提条件

- 汎用インポータではなく `test_04.CSV` 前提の分析スクリプトです。
- 低回転域と高回転終端は仮定値で補完しており、純粋な実測トレースではありません。
- 中盤の `save('custom_engine_curve.mat', 'engine_data')` は `engine_data` 作成前なので、実行順や MATLAB ワークスペース状態に依存する可能性があります。
