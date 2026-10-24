# tire_fitting/models/tmeasy.py

## 役割
統一 `TireModel` インタフェースに適合した新実装の TMeasy モデルです。

## 主な内容
- `calculate_fx()`/`calculate_fy()` は `params` 辞書から係数を取得
- 後半に旧 TMeasy 計算関数を互換維持のため内包

## 背景
旧 `tmeasy_model.py` の数式を保持しつつ、新しいモデル比較フレームワークへ載せ替えるための適応層です。

## 使用ライブラリ
- `numpy`

## パイプライン上の位置
ハイブリッド同定系で使う TMeasy 実装本体です。
