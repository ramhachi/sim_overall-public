# tests/test_end_to_end_integration.py

## 役割
TTC 実データ読み込みから前処理、TMeasy 適合、評価、プロット、YAML/HPP 出力までを 1 本で通す総合統合テストです。

## 守っている振る舞い
- Stage 1 では DriveBrake / Cornering CSV が読め、`FZ` が正値化されることを確認します。
- Stage 2 では DriveBrake の速度・荷重・温度・`SA` 条件、Cornering の荷重条件、IQR 外れ値除去、Low/Medium/High 荷重群分けを確認します。
- Stage 3 では `TireParameterOptimizer` により縦横パラメータが推定され、`mu`・`K`・`C_gamma` が所定レンジに収まることを見ます。
- Stage 4 では `FitEvaluator` で `rmse/nrmse/r2/tic` とピーク摩擦誤差を計算し、目標閾値との比較結果を出します。
- Stage 5 では YAML と C++ ヘッダを書き出し、必須セクションや namespace / constexpr 構造を確認します。

## 主なフィクスチャ/ヘルパー
- `print_section()` が各ステージのログ見出しを整えます。
- `tempfile.mkdtemp()` で一時出力ディレクトリを作り、最後に `shutil.rmtree()` で掃除します。

## 重要なアサーション・生成物
- フィルタ条件の全件 `all()`、荷重帯境界、推定パラメータ bounds、指標値の閾値比較、生成ファイル存在を段階ごとに固定しています。
- 生成物は `test_tire_params.yaml`, `test_tire_params.hpp`, `fx_vs_sr.png`, `fy_vs_sa.png`, `residuals_fx.png`, `residuals_fy.png` です。
- pytest の skip は使わず、データ欠如時は `False` を返して失敗扱いにします。
