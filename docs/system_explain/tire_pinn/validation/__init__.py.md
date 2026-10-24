# `tire_pinn/validation/__init__.py`

## 1. 役割

`tire_pinn.validation` サブパッケージの公開入口です。  
現状は `PhysicsValidator` だけを再公開します。

## 2. 公開 API

- `PhysicsValidator`
  - 学習済みモデルに対しゼロスリップ、対称性、摩擦円、単調性を検査します。

## 3. システム内での位置付け

- `main_pinn_train.py` の deployable 判定はこの validator の結果に依存します。
- 実装本体は `physics_tests.py` です。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import のみです。
