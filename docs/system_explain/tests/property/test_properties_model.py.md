# tests/property/test_properties_model.py

## 役割
旧 `tire_fitting.tmeasy_model.TMeasyModel` の数式実装が、任意入力でも定義式どおりに振る舞うことを検証します。

## 守っている振る舞い
- 縦方向正規化変形量 `xi_bar = Kx * SR / (mu_x * Fz)` を前提に力が計算されること。
- 飽和関数が `s < 1` で `s(2-s)`、それ以外で 1 になること。
- 縦力 `Fx` が `-mu_eff * mu_x * Fz * sign(xi_bar)` に一致すること。
- 荷重依存摩擦係数が `mu_peak * (1 - k_mu*(Fz-Fz_nom)/Fz_nom)` に一致すること。
- 横方向正規化変形量 `eta_bar = Ky * tan(SA) / (mu_y * Fz)` が使われること。
- キャンバ推力 `C_gamma * gamma * Fz` が横力へ加算されること。
- 縦横で同一の飽和関数が使われること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で公称荷重 1000N の `TMeasyModel` を生成します。
- Hypothesis の浮動小数生成に対し、`assume()` で符号未定義や分母ゼロに近いケースを除きます。

## アサーションの意味
手計算値との比較は、実装が内部リファクタ後も物理式から外れないことを保証します。特に飽和関数とキャンバ項は力特性の形を左右するため、ここでの逸脱は車両応答の質的変化に直結します。
