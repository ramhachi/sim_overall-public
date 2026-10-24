# `tire_fitting/models/__init__.py`

## 役割

hybrid 系タイヤモデル群の公開窓口です。  
抽象基底、TMeasy、Magic Formula、Hybrid 合成モデルをまとめて再公開します。

## 実行と入出力

- 直接実行するファイルではありません。
- 入力:
  - `base.py`, `tmeasy.py`, `magic_formula.py`, `hybrid.py`
- 出力:
  - `TireModel`, `TireForces`, `TMeasyModel`, `MagicFormulaModel`, `HybridTireModel`

## 主要要素

### 再公開シンボル
- `TireModel`
- `TireForces`
- `TMeasyModel`
- `MagicFormulaModel`
- `HybridTireModel`

## データフロー

1. 各モデル実装を import します。
2. `__all__` に載せて package API を固定します。

## システム内での位置づけ

本体ではなく **モデル層の入口整理用 glue** です。  
hybrid パイプライン側がモデル種別を統一的に扱えるようにしています。
