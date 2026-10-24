# `tire_pinn/validation/physics_tests.py`

## 1. 役割

学習済み PINN が満たすべき物理拘束を **推論後に採点** する検証器です。  
deployable 判定の根拠となる `pass/metrics/message` 形式の結果を返します。

## 2. クラス `PhysicsValidator`

### `__init__(model, data_frame=None, device="cpu")`
- モデルを eval 化して保持します。
- 参照データがあれば 1〜99 パーセンタイルから入力範囲 `bounds` を作ります。
- 単調性の経験ベース閾値用キャッシュも持ちます。

### `_compute_bounds(data_frame)`
- データが無ければ既定範囲、あれば列別 percentile 範囲を返します。

### `run_all_tests(...)`
- 以下 4 試験を順に実行し、最後に要約を表示します。
  - `test_zero_slip()`
  - `test_symmetry()`
  - `test_friction_circle()`
  - `test_monotonicity()`

## 3. 個別試験

### `test_zero_slip(threshold=20.0)`
- `SR=SA=0` の格子点を作り、`Fx/Fy/Mz` の最大残差が閾値以下か確認します。

### `test_symmetry(mean_threshold=0.25, p95_threshold=0.45)`
- `SA` 正負ペアを比較し、`Fy` 非対称率の平均と p95 を評価します。

### `test_friction_circle(mu_cap=2.6, max_violation_rate=0.05)`
- 入力空間を乱数サンプリングし、`sqrt(Fx²+Fy²) > μFz` の違反率を測ります。

### `test_monotonicity(...)`
- `∂|Fx|/∂Fz`, `∂|Fy|/∂Fz` を autograd で求め、負勾配率を評価します。
- 実データの傾向から `_estimate_empirical_monotonicity_baseline()` で基準を緩和できます。

### `_estimate_empirical_monotonicity_baseline(negative_gradient_tolerance)`
- 実測データをビン分けし、荷重増分に対する `|Fx|`, `|Fy|` の経験的負勾配率を求めます。
- モデルだけでなく実データノイズも考慮するための補助関数です。

### `_print_summary(results)`
- 各試験の PASS/FAIL を一覧表示します。

## 4. 戻り値形式

各試験は共通で以下を返します。

- `pass`: 真偽値
- `metrics`: 閾値や最大違反量などの辞書
- `message`: 人間向け要約文字列

## 5. システム内の位置付け

`main_pinn_train.py` の checkpoint 評価段で使われ、  
R² と並んで「シミュレータへ出してよいか」を決めるゲートです。  
訓練時損失とは別に、独立した後検証として機能します。

## 6. 副作用と注意点

- 標準出力へ詳細ログとサマリを出します。
- 単調性試験は勾配計算を使うため比較的重いです。
- データフレームが無い場合は既定境界でテストします。
