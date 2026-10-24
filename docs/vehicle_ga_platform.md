# LPV車両諸元GAプラットフォーム

## 正規入口

- 対話実行: `EasyStart.m` のメニュー `[3]`
- MATLAB API: `matlab/run_full_vehicle_ga.m`
- 設定SSOT: `matlab/vehicle_ga_default_config.m`
- 物理評価: Release版 `build/cpp/laptime_sim`

通常設定はAutocross、LPV-LQR `steering6`、deployable PINNタイヤ、DRS無効です。
人口80、最大55世代、10 process worker、壁時計5時間、GA本体4時間55分で、
初期集団を含め最大4,480評価を行います。`ga`自体の並列化は使わず、1バッチを
MATLABのprocess poolで並列評価するため、C++プロセスと個体ファイルが衝突しません。

## 探索変数と固定値

設計schemaは`build_full_vehicle_design_schema.m`の46変数を正本とします。
PINN manifestが所有するタイヤ物性7変数は固定し、`apply_full_design`で前後平均に
なる`DamperR`も探索から外して`DamperF`へ従属させます。したがって有効探索次元は
38です。DRSはLPV側に検証済み操作方針がないため、探索にも最終成果物にも含めません。

## 過去解の扱い

`collect_vehicle_ga_seeds.m`は過去artifactからdesign vectorと変数名だけを読みます。
46変数完全一致はそのまま、旧schemaは名前で写像し、欠損値を現schemaの範囲中央、
範囲外を境界値へ移します。過去のラップタイムや評価済みscoreは信用せず、現行の
物理・PINN・LPVで必ず再評価します。読込結果は`seeds/import_report.json`へ保存します。

## 受入ゲート

最終候補は探索時結果を流用せず、CSV trace付きで再評価します。完走、非degraded、
PINN runtime、個体ごとの生成velocity profile、track limit、横偏差、見出し誤差、
操舵追従・飽和、LPV solve failure、LPV実行時間を確認します。並列探索中はOSの
descheduleに左右されにくい平均時間を使い、最大時間は単独で行う最終再評価だけに課します。全ゲート通過時だけ
`output/vehicle_ga_runs/latest_best_vehicle_artifact.json`を更新します。

## 出力と再開

各runは`output/vehicle_ga_runs/<run_id>/`へ隔離します。

- `manifest.json`: 設定、schema、バイナリ/PINN互換性
- `evaluations/batch_*/`: 個体ごとの入力と結果
- `evaluations.jsonl`: 全評価の追記ログ
- `checkpoint/latest.mat`: population、score、乱数状態、互換性契約
- `checkpoint/latest.json`: 人が読める進捗
- `best/`: 最終再評価、trace、vehicle artifact
- `run_summary.json`: runの要約

EasyStartの「最新チェックポイントから再開」は、バイナリ、PINN manifest、schema、
固定値、driver、track、simulation条件が完全一致する場合だけ再開します。

## 保守用コマンド

```matlab
% 5時間run
config = vehicle_ga_default_config(pwd);
result = run_full_vehicle_ga(config);

% 短時間スモーク
config = vehicle_ga_default_config(pwd);
config.quick_mode = true;
result = run_full_vehicle_ga(config);

% 明示したcheckpointから再開
config = vehicle_ga_default_config(pwd);
config.resume_checkpoint = 'output/vehicle_ga_runs/<run_id>/checkpoint/latest.mat';
result = run_full_vehicle_ga(config);
```

単体契約テストは`matlab/tests/test_vehicle_ga_platform.m`です。旧入口だった
`optimize_full_vehicle.m`、`optimize_vehicle_setup.m`、`test_ga.m`、
`debug_optimization.m`は二重運用を避けるため削除済みです。
