# tests/verify_model_calculations.py

## 役割
TMeasy の縦力・横力・荷重依存摩擦係数が、手計算した理論式と一致するかを確認する手動検証スクリプトです。

## 入力と出力
- 入力: スクリプト内で固定した `sr/sa/fz/camber` と係数
- 出力: 中間計算式、期待値、モデル出力、誤差、PASS/FAIL ログ
- 生成物: なし

## 主な処理
- `verify_longitudinal_force()` は通常域と飽和域の 2 ケースで `mu_x`、`xi_bar`、`mu_eff`、`Fx` を手計算し、モデル出力と比較します。
- `verify_lateral_force()` はキャンバなし / ありの 2 ケースで `eta_bar`、`Fy_base`、`Fy_camber` を追います。
- `verify_load_dependency()` は `_compute_load_dependent_friction()` が `mu_peak * (1 - k_mu*(fz-fz_nom)/fz_nom)` に一致するかを確認します。

## 存在理由
- 最適化や前処理ではなく、モデル数式そのものが正しいかを切り出して検証するためのスクリプトです。
- 誤差閾値が `1e-6` や `1e-9` と厳しく、数式実装の回帰を見つけやすくしています。
