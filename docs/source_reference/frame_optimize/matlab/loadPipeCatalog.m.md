# frame_optimize/matlab/loadPipeCatalog.m

## 役割

`pipe_catalog.json` を読み込み、断面特性込みの catalog 配列へ整形する loader です。

## 主な処理

- JSON 読み込み
- `computeTubeSectionProperties` による断面特性計算
- material 情報との整合

## 実装上の注意

- objective では `x(1:5)` の離散 pipe index からこの catalog を参照します。
- saved artifact の `selected_pipes` もこの catalog 情報を基に作られます。
