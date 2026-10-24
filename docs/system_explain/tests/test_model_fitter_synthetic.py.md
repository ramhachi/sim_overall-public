# tests/test_model_fitter_synthetic.py

## 役割
合成データを使って `ModelFitter` の指標計算とパラメータ辞書化が理想条件で破綻しないことを確認するテストです。

## 守っている振る舞い
- `test_calculate_metrics_tmeasy()` は既知 TMeasy 係数で生成した縦力データに対し、`_calculate_metrics()` がほぼ完全一致の指標を返すことを確認します。
- `test_calculate_metrics_magic_formula()` は同じ確認を Magic Formula で行います。
- `test_params_array_to_dict_all_models()` は `tmeasy/magic_formula` × `fx/fy` 全組合せで `_get_parameter_bounds()` と `_params_array_to_dict()` の対応が崩れないことを見ます。

## 主なフィクスチャ/ヘルパー
- fixture はなく、各テストで `ModelFitter` を都度生成します。
- 合成データはモデル自身の `calculate_fx()` で作り、ノイズを入れません。

## 重要なアサーション
- `rmse < 1.0`, `tic < 0.01`, `r2 > 0.99` は、指標実装が真値一致ケースで不自然な値を出さないことの保証です。
- 生成物や skip はなく、長い最適化を避けながら内部計算の正しさを押さえる補助テストです。
