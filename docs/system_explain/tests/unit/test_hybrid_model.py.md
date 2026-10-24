# tests/unit/test_hybrid_model.py

## 役割
`HybridTireModel` が Fx/Fy を別モデルへ委譲しつつ、統一 `TireModel` インターフェースを壊さないことを検証します。

## 守っている振る舞い
- 初期化時に `TireModel` 実装だけを受け付け、異物を `TypeError` で拒否すること。
- `calculate_fx()` は `fx_model`、`calculate_fy()` は `fy_model` の計算結果をそのまま返すこと。
- Fx と Fy が異なるモデルを独立に使えること。
- `get_model_type()` が構成を `hybrid(fx=..., fy=...)` で表すこと。
- `get_parameters()` が両方向の型と係数をまとめて返すこと。
- `set_parameters()` が Fx/Fy の片側または両側だけを更新でき、空 dict では何も変えないこと。
- `calculate_forces()`、`kwargs` 透過、モデル間のパラメータ分離、異なる荷重での一貫性が保たれること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で TMeasy と Magic Formula の代表パラメータを作り、両モデルを生成します。
- 多くのテストが「ハイブリッド結果」と「サブモデル直接結果」を比較します。

## アサーションの意味
このファイルの核心は委譲の正しさです。結果一致アサーションは、ハイブリッド層が余計な変換を挟まず、単に構成と分配だけを担っていることを証明します。
