# `tire_pinn/models/pinn_model.py`

## 1. 役割

PINN タイヤモデルの **ネットワーク本体・正規化ラッパ・ランタイム物理補正ラッパ** を定義する中心モジュールです。

## 2. クラス `TirePINN`

### 役割
- 入力 `[SR, SA, Fz, IA, TSTC]` から出力 `[Fx, Fy, Mx, Mz]` を予測する多層全結合ネットワークです。
- LayerNorm と Swish を採用し、物理損失に必要な滑らかな勾配を確保します。

### `__init__(hidden_dims, input_dim=5, output_dim=4, use_layer_norm=True)`
- `hidden_dims` に従って `Linear -> LayerNorm -> Swish` を積みます。
- 最終層は活性化無しの線形出力です。

### `_initialize_weights()`
- Linear は Xavier uniform、LayerNorm は恒等初期化です。

### `forward(x)`
- 入力 `(batch, 5)` をそのままネットワークへ通します。

### `predict_with_grad(x)`
- 出力と各出力成分の入力勾配 `(batch, 4, 5)` を返します。
- 単調性評価や解析用途向けです。

### `count_parameters()` / `get_architecture_info()`
- パラメータ数や C++ export 用メタ情報を返します。

## 3. クラス `NormalizedTirePINN`

### 役割
- `TirePINN` を Z-score 正規化付き推論器として包みます。

### 主なメソッド
- `set_normalization_params(...)`
  - 入出力平均/標準偏差を buffer へ設定
- `normalize_input(x)`
  - `(x - mean) / std`
- `denormalize_output(y_norm)`
  - `y_norm * std + mean`
- `forward(x)`
  - raw 入力 → 正規化 → PINN → raw 出力
- `forward_normalized(x_norm)`
  - 正規化済み入力をそのまま core PINN へ流す

## 4. クラス `PhysicsEnforcedTireModel`

### 役割
- 学習済み `NormalizedTirePINN` に軽量後処理を被せ、ランタイム側での物理破綻を抑えます。

### 主な補正
- `_apply_zero_slip_correction()`
  - SR/SA がゼロ近傍なら residual Fx/Fy/Mz を打ち消す
- `_apply_symmetry_projection()`
  - SA 鏡像入力を使って Fy/Mz の反対称性を補正
- `_apply_sa_zero_fy_correction()`
  - `SA -> 0` で Fy をガウシアンゲートにより 0 へ落とす
- `forward(x)`
  - 上記補正後、必要なら `μFz` で摩擦円キャップをかける
- `get_runtime_postprocess_config()`
  - C++ 側へ複製すべき後処理設定を辞書で返す

## 5. データフローとシステム適合

- 学習時の中心表現は `NormalizedTirePINN`
- export/実行時の安全版表現は `PhysicsEnforcedTireModel`

という二層構造です。  
この分離により、学習自由度とランタイム安全性を両立しています。

## 6. 副作用と注意点

- 正規化係数は buffer として state に含まれます。
- `PhysicsEnforcedTireModel` は推論結果を後処理で変更するため、純粋な学習面と実行面の出力が一致しない場合があります。
