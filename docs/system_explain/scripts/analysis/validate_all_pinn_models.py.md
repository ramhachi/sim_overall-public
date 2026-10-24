# `scripts/analysis/validate_all_pinn_models.py`

## 1. 役割

学習済み PINN タイヤモデル一式を対象に、**実データ一致度** と **ゼロスリップ近傍の物理リーク** をまとめて検証する総合バリデーションスクリプトです。  
特に「DriveBrake 条件で Fy が漏れるか」を重点確認します。

## 2. 入出力

- 実行例:
  - `python3 scripts/analysis/validate_all_pinn_models.py --device cpu`
  - `python3 scripts/analysis/validate_all_pinn_models.py --tire Hoosier_18p0X6p0-10_R20`
- 主な入力:
  - `output/tire_models/<tire>/manifest.json`
  - `output/tire_models/registry.json`
  - `tire_pinn/config/pinn_config.yaml`
  - TTC データ
  - 学習済み checkpoint
- 主な出力:
  - `output/validation/<tire>/pred_vs_actual.png`
  - `fy_leakage_drivebrake.png`
  - `fy_vs_sa.png`
  - `fx_vs_sr.png`
  - `zero_slip_check.png`
  - `residuals.png`
  - 標準出力の数値サマリ

## 3. 主要制御フロー

1. `main()` が device と対象タイヤを決めます。
2. `registry.json` から deployable モデル一覧を取得します。
3. 各タイヤごとに `validate_one()` を呼びます。
4. `load_model_from_manifest()` が manifest と config を読んでモデルを復元し、必要なら `PhysicsEnforcedTireModel` で後処理ラップします。
5. 各種 plot 関数が比較図・リーク図・残差図を順番に保存します。
6. 最後に出力ディレクトリ一覧を表示します。

## 4. 関数

### `load_model_from_manifest(tire_name: str, device: str)`
- 役割: manifest から checkpoint と設定を見つけ、推論用モデルを復元します。
- 主な処理:
  - `selected_checkpoint` が無ければ `best_r2_model.pt` へフォールバック
  - `TTCDataLoader(normalize=True)` で正規化係数取得
  - `TirePINN -> NormalizedTirePINN` を構築
  - runtime postprocess 有効時は `PhysicsEnforcedTireModel` を被せる
- 戻り値: `(model, loader, manifest)`

### `_r2(actual, pred) -> float`
- 残差平方和と全変動から R² を計算します。

### `_rmse(actual, pred) -> float`
- RMSE を返します。

### `_predict(model, x_np, device) -> np.ndarray`
- NumPy 配列を tensor 化し、モデル推論結果を NumPy へ戻します。
- `@torch.no_grad()` 付きで勾配計算はしません。

### `plot_pred_vs_actual(model, loader, device, out_dir, tire_name)`
- 役割: Fx/Fy/Mx/Mz の予測 vs 実測散布図を 2x2 で作ります。
- 特徴:
  - `source_test_type` があれば `Db/Cr/Cmb` 別に色分け
  - 各軸に R² と RMSE を注記
  - 標準出力へソース別集計も表示

### `plot_fy_leakage_drivebrake(model, loader, device, out_dir, tire_name)`
- 役割: DriveBrake 条件で Fy が漏れていないか検査します。
- 作る図:
  1. `SA` に対する `Fy_pred`
  2. `|SA|<1°` における `|Fy|` ヒストグラム
  3. `SA=0` のまま `SR` を掃引したときの Fy
- 数値出力: 平均絶対 Fy、95 パーセンタイル、20N 超過率など

### `plot_fy_vs_sa(model, loader, device, out_dir, tire_name)`
- 役割: `SR=0` での横力曲線と、実コーナリングデータのオーバーレイを描きます。
- 重点: `SA=0` の零交差が崩れていないか確認します。

### `plot_fx_vs_sr(model, loader, device, out_dir, tire_name)`
- 役割: `SA=0` での縦力曲線と DriveBrake 実データを比較します。
- 重点: `SR=0` 近傍のゼロ点と負荷別カーブ形状です。

### `plot_zero_slip_check(model, loader, device, out_dir, tire_name)`
- 役割: 最重要のゼロスリップ物理チェックです。
- 3 行構成:
  1. `SR=SA=IA=0` での `Fx/Fy/Mz` vs `Fz`
  2. `SA=0` のまま `SR` 掃引した時の Fy
  3. `SR=0` のまま `SA` 掃引した時の Fx
- 標準出力には荷重別リーク表も出します。

### `plot_residuals(model, loader, device, out_dir, tire_name)`
- 役割: Fx/Fy 残差を `SA`, `SR`, `Fz` などに対して可視化します。
- 末尾パネルでは `source_test_type` 別の Fy 残差箱ひげ図も作ります。

### `validate_one(tire_name: str, device: str) -> dict`
- 役割: 1 タイヤ分の全検証を実行します。
- 副作用: 対象用の出力ディレクトリを作り、全 PNG を保存します。
- 戻り値: タイヤ名と出力ディレクトリ。

### `main()`
- 役割: CLI 入口。
- 補足:
  - `mps` / `cuda` が使えない場合は CPU にフォールバック
  - `--tire` 未指定時は deployable な全モデルを対象

## 5. 注意点

- `ROOT = Path(__file__).parent` を `sys.path` 先頭へ入れているため、実行位置依存を緩和しています。
- 描画バックエンドは `Agg` 固定で、GUI なし環境前提です。
- `TireCatalog` は import されていますが、このファイル内では未使用です。
