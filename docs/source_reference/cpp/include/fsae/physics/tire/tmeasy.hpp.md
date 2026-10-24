# tmeasy.hpp

- **目的**: 緩和長付き TMeasy タイヤモデルを実装します。
- **主要要素**: `TMeasyParams`、`TMeasyState`、`TMeasyOutput`、`compute_tmeasy()`、`compute_tmeasy_batch()`。
- **理論背景**: 変形量 `xi/eta` の一次遅れと摩擦飽和で縦横力と SAT を近似します。
- **依存**: `cmath`、`algorithm`。
- **システム上の位置づけ**: 標準タイヤモデルであり、多くの車両モデルの既定選択です。

