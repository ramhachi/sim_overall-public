# `tire_pinn/models/__init__.py`

## 1. 役割

`tire_pinn.models` サブパッケージの公開入口です。  
PINN 本体と Swish 系活性化を再公開します。

## 2. 公開 API

- `TirePINN`
- `NormalizedTirePINN`
- `SwishActivation`
- `SwishLUT`

## 3. システム内での位置付け

- 学習側・検証側・出力側から参照されるモデル API を集約します。
- 実装の中心は `pinn_model.py`、活性化近傍は `activations.py` です。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import のみです。
