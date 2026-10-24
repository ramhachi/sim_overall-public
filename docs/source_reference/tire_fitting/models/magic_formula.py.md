# tire_fitting/models/magic_formula.py

## 役割
簡略 Pacejka Magic Formula を `TireModel` として実装します。

## 主な内容
- `calculate_fx()`: `pDx1/pCx1/pEx1/pKx*` と温度補正を使う縦力
- `calculate_fy()`: `pDy1/pCy1/pEy1/pKy*` とキャンバー補正を使う横力
- `_magic_formula()`: 共通正弦-アークタンジェント形

## 背景・理論
Magic Formula は経験式モデルで、係数 `B,C,D,E` によりピーク・形状・曲率を柔軟に再現します。本実装はリポジトリの輸出フォーマットと比較用途に必要な最小部分へ絞っています。

## 使用ライブラリ
- `numpy`

## パイプライン上の位置
TMeasy と競合させる高柔軟モデル候補です。
