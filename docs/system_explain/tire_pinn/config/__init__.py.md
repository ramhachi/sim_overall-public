# `tire_pinn/config/__init__.py`

## 1. 役割

`tire_pinn.config` を Python パッケージとして成立させるための最小ファイルです。  
内容は docstring のみで、設定読込ロジックや公開 API は持ちません。

## 2. 実行上の位置付け

- `tire_pinn/config/` 配下の YAML や設定関連資産を import 階層に載せるための境界です。
- 実設定の意味付けは `main_pinn_train.py` 側が担当します。

## 3. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: なし
