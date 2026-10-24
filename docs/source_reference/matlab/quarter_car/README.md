# `matlab/quarter_car/`

1/4 車両サスペンションの基礎検証用ディレクトリ。  
Bouc-Wen ダンパを含む簡易縦振動モデルを MATLAB で積分し、CSV/PNG 出力や C++ との比較に使う。

## 収録ファイル

- `quarter_car_ode.m.md` — 5 状態の quarter-car ダイナミクス本体
- `run_and_export.m.md` — 実行と CSV / 画像出力
- `run_verification.m.md` — MATLAB と C++ の比較検証

## このディレクトリの役割

サスペンション縦系の最小検証基盤として、  
より大きな 8DOF / 14DOF モデルへ進む前の基準応答を提供する。
