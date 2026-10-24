# tests/property/test_properties_tmeasy_compat.py

## 役割
新しい `tire_fitting.models.tmeasy.TMeasyModel` が、旧実装と後方互換な力計算とパラメータ API を持つことを確認します。

## 守っている振る舞い
- 任意の妥当な係数・入力で、縦力が旧実装と数値誤差内で一致すること。
- 横力も同様に一致すること。
- ベクトル化旧実装と、新実装の逐次計算結果が一致すること。
- 新実装の `get_model_type()` が `tmeasy` を返すこと。
- `get_parameters()` / `set_parameters()` の往復で値更新と未変更値保持が両立すること。

## 主なフィクスチャ/ヘルパー
- `tire_parameters()` と `tire_inputs()` が Hypothesis 用のパラメータ・入力セットを生成します。
- 旧実装 `OriginalTMeasy` と新実装 `NewTMeasy` を並列比較します。

## アサーションの意味
`np.isclose` / `np.allclose` による比較は、実装差分が API 層だけで計算結果に影響しないことの保証です。移行期に既存フィット結果やシミュレーション応答を変えないための重要な回帰テストです。
