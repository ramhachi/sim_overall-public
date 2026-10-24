# tests/test_model_fitter_integration.py

## 役割
`ModelFitter.fit_multiple_models()` を実データで動かし、結果構造・指標・キャッシュ・並列実行を確認する統合テストです。

## 守っている振る舞い
- `test_fit_tmeasy_longitudinal()` は DriveBrake から `tmeasy/fx` を適合し、`params` と `metrics` に必要キーが揃うことを確認します。
- `test_fit_tmeasy_lateral()` は Cornering で `tmeasy/fy` を適合し、横力側のキーが揃うことを見ます。
- `test_result_caching()` は同条件を 2 回実行したとき `fitter.cache['tmeasy_fx']` が使われ、結果が完全一致することを確認します。
- `test_fit_multiple_models_parallel()` は `tmeasy` と `magic_formula` を `fx/fy` 両方向で同時実行し、全結果が揃うことを確認します。

## 主なフィクスチャ/ヘルパー
- 先頭で `DRIVEBRAKE_FILE` と `CORNERING_FILE` の存在を判定し、各テストはその結果を共有します。
- 各テストで `max_workers` を変え、逐次実行と並列実行の両方を通します。

## 重要なアサーション・スキップ
- `rmse > 0`, `0 <= tic <= 1`, `r2 <= 1` など、結果値が最低限妥当な範囲にあることを見ます。
- 実データが無い場合は `@pytest.mark.skipif` でスキップします。
- 生成物はファイルではなく、stdout に出る比較ログと `ModelFitter` 内キャッシュです。
