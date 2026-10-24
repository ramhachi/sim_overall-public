# tests/property/test_properties_hybrid.py

## 役割
`HybridTireModel` のモデル種別問い合わせが、どの組み合わせでも一貫して正しい文字列とパラメータ表現を返すかを検証します。

## 守っている振る舞い
- `TMeasyModel.get_model_type()` が常に `tmeasy` を返すこと。
- `MagicFormulaModel.get_model_type()` が常に `magic_formula` を返すこと。
- `HybridTireModel.get_model_type()` が `hybrid(fx=..., fy=...)` 形式で正確に両方向を表すこと。
- パラメータ更新後もモデル種別自体は変化しないこと。
- `fx` と `fy` を入れ替えたハイブリッドが別の型文字列になること。
- `get_parameters()` の `fx_model`/`fy_model` も問い合わせ結果と整合すること。

## 主なフィクスチャ/ヘルパー
- `tmeasy_params_strategy()` と `magic_formula_params_strategy()` が妥当な係数集合を作ります。
- `model_type_strategy` が `tmeasy` / `magic_formula` の組み合わせを網羅します。

## アサーションの意味
このテスト群は「中身の係数値」と「モデルの種別情報」を分離して守っています。ハイブリッド構成の識別が壊れると、保存形式・ログ・検証レポートで誤モデル扱いになるため、その防波堤です。
