# `tire_pinn/training/trainer.py`

## 1. 役割

PINN の訓練・検証・checkpoint 管理を行う **学習ループ本体** です。  
データ損失と物理損失を curriculum に従って合成し、早期終了とベストモデル保存まで担当します。

## 2. 入出力

- 主な入力:
  - `NormalizedTirePINN`
  - train/val `DataLoader`
  - `CollocationSampler`
  - 学習率、weight decay、出力先
- 主な出力:
  - `best_model.pt`
  - `best_r2_model.pt`
  - `final_model.pt`
  - `phase_*.pt`
  - `training_history.json`

## 3. クラス `PINNTrainer`

### `__init__(...)`
- optimizer は Adam、LR scheduler は cosine annealing です。
- `DataLoss` と各物理損失を生成し、`PhysicsLossManager` と `CurriculumScheduler` を組み立てます。

### `set_normalization_params_for_physics(data_loader)`
- `TTCDataLoader` の統計を読み、摩擦円損失へ反映します。
- ここで学習状態 `epoch`, `best_*`, `history` も初期化します。

### `_unpack_batch(batch)`
- `(x, y, component_mask)` または `(x, y)` の両形式を吸収します。

### `train_epoch()`
- 1 epoch 分の学習本体です。
- 流れ:
  1. curriculum 重み取得
  2. データ損失計算
  3. Phase B 以降は collocation を生成
  4. `PhysicsLossManager.compute_all()` で物理損失計算
  5. 合計損失を逆伝播
  6. 勾配クリップ後に optimizer step

### `validate()`
- 検証セットで raw 出力の損失と R² を計算します。
- さらに別途 collocation を生成し、物理損失系メトリクスも集めます。
- 単調性は `@torch.no_grad()` と両立しないため省略されます。

### `save_checkpoint(filename)` / `load_checkpoint(filename)`
- モデル、optimizer、scheduler、履歴、ベスト値を保存/復元します。

### `train(max_epochs=260, patience=20, val_interval=5)`
- 学習全体の制御関数です。
- 監視指標は phase に応じて切り替え、改善時だけ `best_model.pt` を更新します。
- `val_r2_mean` の最高値は別に `best_r2_model.pt` へ保存します。

### `_save_history()`
- 履歴辞書を JSON 保存します。

## 4. データフローとシステム適合

`TTCDataLoader` の実測データと `CollocationSampler` の人工点を同時に扱い、  
`CurriculumScheduler` の時間計画に従って最適化を回す、`tire_pinn` 学習部の中心です。

## 5. 副作用と注意点

- checkpoint と履歴 JSON をファイル保存します。
- `tqdm` による進捗表示を行います。
- `set_normalization_params_for_physics()` を呼ぶ前提で内部状態が初期化されます。
