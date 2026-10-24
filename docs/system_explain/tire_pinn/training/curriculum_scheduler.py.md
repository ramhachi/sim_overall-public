# `tire_pinn/training/curriculum_scheduler.py`

## 1. 役割

PINN 学習を 4 段階で進める **curriculum scheduler** です。  
データ適合を先に固め、後から物理拘束を強めることで損失競合を和らげます。

## 2. フェーズ構成

- Phase A: データ損失のみ
- Phase B: ゼロスリップ・対称性・`SA=0` Fy 抑制を導入
- Phase C: 摩擦円・単調性も段階投入
- Phase D: 最終重みで固定

## 3. 主要メソッド

### `__init__(total_epochs=260, data_weight=1.0, final_weights=None)`
- `total_epochs` に合わせて境界 epoch を自動スケールします。
- `final_weights` 未指定時は `zero/sym/fric/mono/fy_sa_zero` の既定値を使います。

### `_scale_phase_boundaries(total_epochs)`
- 260 epoch 基準の境界を現在の epoch 数へ射影します。

### `get_weights(epoch)`
- 現 epoch の重み辞書を返します。
- 戻り値キー: `data`, `zero`, `sym`, `fric`, `mono`, `fy_sa_zero`

### `get_phase(epoch)`
- `A/B/C/D` を返します。

### `get_phase_info(epoch)`
- フェーズ名、フェーズ内進捗、重みをまとめて返します。

### `should_validate(epoch, val_interval=5)`
- 通常周期に加え、フェーズ境界でも必ず検証します。

### `get_early_stopping_metric(epoch)`
- Phase A/B は `val_r2_mean`、Phase C/D は `val_physics_loss_total` を監視します。

### `print_schedule()`
- 代表 epoch の重み表を標準出力へ出します。

## 4. システム内の位置付け

`PINNTrainer` は毎 epoch このスケジューラから重みと監視指標を取得します。  
そのため本ファイルは「学習の時間軸設計」を担います。

## 5. 副作用

- `print_schedule()` 使用時のみ標準出力へ表示します。
