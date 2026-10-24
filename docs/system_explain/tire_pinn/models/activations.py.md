# `tire_pinn/models/activations.py`

## 1. 役割

PINN 用の Swish 活性化と、その近似版 LUT を定義します。  
滑らかな勾配を保ち、物理損失での自動微分を安定させる狙いがあります。

## 2. クラス

### `SwishActivation`
- 数式: `x * sigmoid(x)`
- 特徴:
  - C∞ 連続
  - ReLU より勾配が滑らか
  - 学習時の標準活性化

### `SwishLUT(table_size=2048, x_min=-8.0, x_max=8.0)`
- Swish をテーブル近似で評価します。
- `lut` buffer を事前計算し、`forward()` で clamp → index 化 → 線形補間します。
- 用途は「高速推論/検証/書き出し補助」で、学習自体は厳密 Swish を使います。

## 3. システム内での位置付け

- `TirePINN` は隠れ層活性化として `SwishActivation` を使います。
- C++ 側実装や近似評価を考える際の基礎定義です。

## 4. 副作用

- `SwishLUT` は初期化時に lookup table を buffer 登録します。
- それ以外の副作用はありません。
