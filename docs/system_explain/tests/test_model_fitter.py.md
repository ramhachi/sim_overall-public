# tests/test_model_fitter.py

## 役割
`ModelFitter` の公開 API と内部補助メソッドの最小契約を確認する単体テストです。

## 守っている振る舞い
- `test_model_fitter_instantiation()` は `data_loader`、`preprocessor`、空の `cache` を持つインスタンスが作れることを確認します。
- `test_model_fitter_has_required_methods()` は `fit_multiple_models()` と内部メソッド群が存在することを固定します。
- `test_get_parameter_bounds_*()` は TMeasy / Magic Formula の縦横 bounds 本数と `(min,max)` 形式を確認します。
- `test_params_array_to_dict_*()` は配列から名前付き辞書への写像が方向ごとに正しいことを見ます。
- `test_fit_multiple_models_validation()` は `fx` に DriveBrake、`fy` に Cornering を渡さないと `ValueError` になることを確認します。

## 主なフィクスチャ/ヘルパー
- fixture はなく、各テストで `TireDataLoader` と `TireDataPreprocessor` を作って `ModelFitter` を初期化します。
- 一部テストで `numpy` 配列を使い、内部変換の入力を直接組みます。

## 重要なアサーション
- ここで見ているのは最適化精度ではなく、API 面の契約とパラメータ名対応の正しさです。
- 実データ依存も skip もなく、軽量に失敗原因を切り分けられる基礎テストになっています。
