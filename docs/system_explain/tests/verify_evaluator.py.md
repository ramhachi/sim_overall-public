# tests/verify_evaluator.py

## 役割
`FitEvaluator` の指標計算と描画 API を、合成データで一通り試す確認用スクリプトです。

## 入力と出力
- 入力: スクリプト内で生成する 100 点の合成 `y_true/y_pred`、一定荷重 `fz_data`、滑り量配列
- 出力: `rmse/nrmse/r2/tic`、ピーク摩擦誤差、品質解釈ログ
- 生成物: `output/test_force_vs_slip.png`, `output/test_residuals.png`

## 主な処理
- `np.random.seed(42)` を固定し、真値と予測値の 2 系列を合成します。
- `calculate_metrics()` と `calculate_peak_friction_error()` の結果を閾値付きで解釈します。
- `plot_force_vs_slip()` と `plot_residuals()` を `Agg` backend で実行し、ヘッドレス環境でも図保存できるか確認します。

## 存在理由
- 評価器の数値 API と描画 API を切り離して確認できる簡易検証ツールです。
- 実データが無くても evaluator 単体の故障を見つけられます。
