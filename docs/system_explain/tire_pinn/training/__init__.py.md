# `tire_pinn/training/__init__.py`

## 1. 役割

`tire_pinn.training` の公開入口です。  
学習ループ、collocation サンプラ、curriculum scheduler をまとめて再公開します。

## 2. 公開 API

- `PINNTrainer`
- `CollocationSampler`
- `CurriculumScheduler`

## 3. システム内での位置付け

- `main_pinn_train.py` はこの層のクラス群を組み合わせて訓練フェーズを構成します。
- 実ロジックは各下位ファイルにあります。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: import のみです。
