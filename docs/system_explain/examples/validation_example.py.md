# `examples/validation_example.py`

## 1. 役割

タイヤフィッティング品質検証 API の使い方を示すサンプルです。  
良好ケース、不良ケース、閾値変更、ハイブリッド構成の 4 例を通して `tire_fitting.validation` の戻り値を確認できます。

## 2. 入出力

- 実行例: `python3 examples/validation_example.py`
- 入力: スクリプト内に埋め込んだ疑似 `fitting_results`
- 出力:
  - 検証結果の標準出力
  - テキストレポート

## 3. 関数

### `example_good_fitting()`
- 良好な `fx`/`fy` 指標を与え、全体合格と詳細レポートを表示します。

### `example_poor_fitting()`
- RMSE/TIC/R² が閾値を外れる例を作り、`get_failed_metrics()` の利用例を示します。

### `example_custom_thresholds()`
- 同じ結果に対して
  - デフォルト閾値
  - 緩い閾値
  - 厳しい閾値
  の 3 通りで判定が変わる様子を示します。

### `example_hybrid_model()`
- `fx=magic_formula`, `fy=tmeasy` の組合せを与え、各 validation オブジェクトの詳細値を参照する例です。

## 4. トップレベルフロー

1. `logging.basicConfig()` で INFO ログ形式を設定します。
2. `__main__` で 4 つの例を順番に実行します。
3. 最後に完了メッセージを表示します。

## 5. 注意点

- 実データ処理は行わず、検証 API の使い方説明に特化したサンプルです。
- 同一内容の関数が複数例で使われるため、比較しやすいように出力区切り線を多用しています。
