# nn_lookup_tire.hpp

- **目的**: MATLAB/JSON から書き出したニューラルネットタイヤモデルを読み込み、推論します。
- **主要要素**: `TireForces`、`TireInput`、`FCLayer`、`BatchNormLayer`、`RuntimeLayer`、`NNTextTireModel`。
- **理論背景**: 正規化・全結合層・LayerNorm/Swish/ReLU・後処理を含む推論を C++ 単独で再現します。
- **依存**: `filesystem`、`nlohmann/json`、`fstream`。
- **システム上の位置づけ**: PINN/NN ベースタイヤモデル実行層です。

