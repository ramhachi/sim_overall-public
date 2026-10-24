# tests/property/test_properties_optimizer.py

## 役割
`TireParameterOptimizer` がランダムな合成データに対しても、仕様境界内の妥当な推定結果を返すかを確認します。

## 守っている振る舞い
- 縦フィット結果の `mu_x_peak`、`k_mu_x`、`Kx` が許容範囲内に収まること。
- 横フィット結果の `mu_y_peak`、`k_mu_y`、`Ky`、`C_gamma` が許容範囲内に収まること。
- 返却 dict にパラメータだけでなく `rmse`、`r2`、`success`、`message` が含まれること。
- `rmse` が非負、`r2` が 1 以下であること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で `TMeasyModel` と `TireParameterOptimizer` を生成します。
- Hypothesis が真値パラメータと乱数 seed を生成し、その真値から合成力データを作ります。

## アサーションの意味
ここでは「真値へ完全一致するか」よりも、「最適化器が異常値や欠損キーを返さないか」を重視しています。探索が失敗しても API 契約を守り、後続の評価・保存工程が破綻しないことを担保するテストです。
