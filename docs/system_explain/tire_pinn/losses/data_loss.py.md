# `tire_pinn/losses/data_loss.py`

## 1. 役割

実測 TTC データとの一致度を測る **教師データ損失** と **R² 指標** を定義します。

## 2. クラス `DataLoss`

### 役割
- Fx/Fy/Mx/Mz の成分別 MSE を計算し、重み付き合計を返します。
- Db/Cr の片側データに対応するため `component_mask` を受け取れます。

### `__init__(component_weights=[1.0, 1.0, 0.5, 0.5], normalize=True)`
- `component_weights`: `[Fx, Fy, Mx, Mz]` の重み
- `normalize=True`: 有効サンプル数で平均化します

### `forward(pred, target, component_mask=None)`
- 入力:
  - `pred`, `target`: `(batch, 4)`
  - `component_mask`: `(batch, 4)` または `None`
- 戻り値:
  - `total_loss`
  - `metrics` 辞書
- `component_mask` がある成分だけ MSE 集計に使います。

## 3. クラス `R2Score`

### `compute(pred, target, component_mask=None)`
- 成分別 R² を tensor で返します。
- マスク有効時は各成分の active サンプルだけで計算します。

### `compute_dict(pred, target, component_mask=None)`
- `r2_fx`, `r2_fy`, `r2_mx`, `r2_mz` の辞書へ変換します。

## 4. データフローとシステム適合

- `PINNTrainer.train_epoch()` は正規化空間で `DataLoss` を使います。
- `main_pinn_train.py` と `PINNTrainer.validate()` は `R2Score` で採点します。
- `component_mask` により DriveBrake / Cornering 混在データでも 1 つのモデルを訓練できます。

## 5. 副作用

- 副作用はありません。
- `weights` は buffer 登録され、state と一緒にデバイス移動します。
