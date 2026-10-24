# `tire_pinn/__init__.py`

## 1. 役割

`tire_pinn` パッケージの公開入口です。  
PINN タイヤモデルの **モデル本体・データローダ・学習器・損失・物理検証・C++ 書き出し** をまとめて再公開します。

## 2. 公開 API

- `__version__ = '0.1.0'`
- モデル:
  - `TirePINN`
  - `NormalizedTirePINN`
- データ:
  - `TTCDataLoader`
  - `TireCatalog`
- 学習:
  - `PINNTrainer`
  - `CollocationSampler`
  - `CurriculumScheduler`
- 損失/検証/出力:
  - `DataLoss`
  - `PhysicsLossManager`
  - `PhysicsValidator`
  - `CPPExporter`

## 3. システム内での位置付け

- `import tire_pinn` だけで主要部品へ触れられるようにする薄い集約層です。
- 実処理は各下位パッケージにあり、このファイル自体は学習や推論を実行しません。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import 時に下位モジュールを読み込み、`__all__` を構成します。
