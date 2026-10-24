# `matlab/dl_tire_model/diagnose_drivebrake_data.m`

## 1. 概要

DriveBrake 系 TTC データの分布とフィルタ条件を診断するスクリプトです。  
Run ごとの統計、`SA` 分布、`|SA|` 閾値別の残存データ品質、可視化、推奨閾値の算出までを 1 本で実行します。

## 2. ファイルの役割

- `clean_and_retrain.m` で問題になった `db_sa_threshold=1.0` の影響を定量確認します。
- `SR` と `Fx` の相関、`SA` 分布、`Fz` 条件ごとの残り方を見て、DriveBrake 用フィルタ戦略を決めるための診断材料を出します。
- 解析結果の図を `output/diagnose_drivebrake_<tire_model>.png` に保存します。

## 3. インターフェース

### スクリプトとしての入出力

- 入力
  - 直接引数はありません。
  - スクリプト内で `tire_model = 'Hoosier_18p0X6p0-10_R20'`、`data_dir = '../../data/tiredata/csv'` を設定します。
- 主な生成物
  - コンソール出力による診断結果
  - Figure ウィンドウ
  - `output/diagnose_drivebrake_<tire_model>.png`
- 主な作業変数
  - `db_names`, `run_data`, `db_all`
  - `thresholds`, `d1`, `d3`, `d5`
  - `best_th`, `best_corr`, `best_n`

## 4. 依存関係

- 独自関数: `loadTireData`
- MATLAB 標準機能: `dir`, `sort`, `fullfile`, `corr`, `prctile`, `unique`, `histogram`, `scatter`, `boxplot`, `saveas`, `mkdir`
- データ表には少なくとも `SA`, `SR`, `Fx`, `Fy`, `Fz` 列が必要です。

## 5. 処理の流れ

### セクション 1: データ読み込み

1. タイヤ名と CSV ディレクトリを固定文字列で設定します。
2. `Db_<tire_model>*.csv` に一致する DriveBrake ファイル一覧を取得します。

### セクション 2: Phase 1 - Run 毎の分析

1. 各 CSV を `loadTireData` で読み込みます。
2. Run ごとに `SA/SR` の平均、標準偏差、範囲を集計して表示します。
3. 読み込んだテーブルを `run_data`、ファイル名を `run_labels` に保持します。

### セクション 3: Phase 2 - `SA` 分布の詳細分析

1. 全 Run を結合して `db_all` を作ります。
2. `SA` の平均、標準偏差、中央値、分位点を出力します。
3. `0.1` 度刻みに丸めたユニーク値数を出し、少数なら一覧も表示します。
4. 固定ビン `[-15,-10,-7,-5,-3,-1,-0.5,0.5,1,3,5,7,10,15]` ごとに件数と `Fx/SR` 統計を表示します。

### セクション 4: Phase 3 - `|SA|` 閾値別評価

1. `thresholds = [0.5,1,2,3,5,7,10,15]` を順に試します。
2. 各閾値で `abs(SA) < th` を満たすデータだけ残し、件数・残存率・`corr(SR,Fx)`・`Fx/Fy/SR` 統計を出します。

### セクション 5: Phase 4 - フィルタ有無の `SR vs Fx`

1. 全データ、`|SA|<1°`、`|SA|<3°`、`|SA|<5°` を比較します。
2. 各条件で `corr(SR,Fx)`、`Fx` 範囲、`SR` 範囲を表示します。

### セクション 6: Phase 5 - `Fz` 条件別確認

1. `Fz_bins = [0,300,600,900,1200,1500,2000]` を使います。
2. `|SA|<1°` で絞ったデータを `Fz` 区間ごとに分け、`SR/Fx` の範囲と `Fx std` を出します。

### セクション 7: Phase 6 - 可視化

1. 3×4 の subplot に `SA/SR/Fx/Fy/Fz` の分布や散布図を配置します。
2. `run_data` を使って Run ごとの `SA` boxplot も作ります。
3. `output` ディレクトリが無ければ作成し、PNG を保存します。

### セクション 8: Phase 7 - 推奨事項

1. 各閾値について `abs(corr(SR,Fx)) * sqrt(N)` をスコアとして比較します。
2. `height(d) > 1000` を満たす閾値だけ候補にし、最良閾値を推奨として表示します。
3. `|SA|<1°` で `SR` 範囲が 0.1 未満なら警告を出します。

## 6. 関数・ローカル関数詳細

このファイルはスクリプトで、ローカル関数は定義していません。  
処理は上記 8 セクションに分かれ、各段階の結果を後続セクションでも再利用します。

## 7. 注意点

- 対象タイヤ名とデータディレクトリはコード内固定です。
- 推奨閾値は `corr(SR,Fx)` とデータ数のバランスで決めており、別の品質指標は使っていません。
- `xticklabels` では `run_labels` の末尾 2 文字を切り出しているため、ファイル名形式が変わると表示が崩れる可能性があります。

