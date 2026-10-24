# `tire_pinn/losses/__init__.py`

## 1. 役割

`tire_pinn.losses` サブパッケージの公開窓口です。  
データ損失、R²、各物理損失、損失管理器をまとめて再公開します。

## 2. 公開 API

- `DataLoss`
- `R2Score`
- `ZeroSlipLoss`
- `SymmetryLoss`
- `FrictionCircleLoss`
- `MonotonicityLoss`
- `PhysicsLossManager`

## 3. システム内での位置付け

- 学習ループ側はこのパッケージ経由で損失群を引き込みます。
- 実装本体は `data_loss.py` と `physics_losses.py` に分かれています。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import のみです。
