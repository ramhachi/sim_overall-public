# `tire_pinn/data/__init__.py`

## 1. 役割

`tire_pinn.data` サブパッケージの公開入口です。  
データ取り込み層から主に使う 2 つの API、`TTCDataLoader` と `TireCatalog` を再公開します。

## 2. 公開 API

- `TTCDataLoader`
  - TTC CSV / raw `.dat` を学習用 DataFrame と DataLoader に変換します。
- `TireCatalog`
  - 利用可能タイヤデータセットを走査し、Db/Cr/Cmb の被覆を整理します。

## 3. システム内での位置付け

- `main_pinn_train.py` や利用側コードが `from tire_pinn.data import ...` と短く書けるようにする薄い再公開層です。
- 実処理は `ttc_loader.py` と `tire_catalog.py` にあります。

## 4. 入出力と副作用

- 入力: なし
- 出力: なし
- 副作用: 下位モジュール import のみです。
