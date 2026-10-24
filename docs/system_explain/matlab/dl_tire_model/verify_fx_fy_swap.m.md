# `matlab/dl_tire_model/verify_fx_fy_swap.m`

## 1. 概要

`verify_fx_fy_swap.m` は、データパイプラインや学習済みモデルの中で `Fx` と `Fy` が入れ替わっていないかを多段階で検証するスクリプトです。生データ相関、`loadTireData` 後の整合、正規化パラメータ比較、モデル出力の入替シミュレーション、ゼロ近傍データ分析、可視化、最終判定を行います。

## 2. ファイルの役割

このスクリプトの役割は、`Fx/Fy` 入替仮説を段階的に切り分けることです。コメントでは以下の Phase が明示されています。

- Phase 1: 生 CSV データの物理的整合性チェック
- Phase 2: `loadTireData → extractTargets` パイプライン検証
- Phase 3: 正規化パラメータ整合性チェック
- Phase 4: モデル出力の Fx/Fy 入替シミュレーション
- Phase 5: 生データの SA/SR 相関分析

実際のコードではさらに可視化と最終判定まで進みます。

## 3. インターフェース

### スクリプトとしての入力

- 既定タイヤ名
  - `tire_model = 'Hoosier_18p0X6p0-10_R20'`
- 生 CSV 検索先
  - `csv_dir = '../../data/tiredata/csv'`
- 必要な学習済みモデル
  - `output/cornering_<tire_model>.mat`
  - `output/drivebrake_<tire_model>.mat`

### スクリプトとしての出力

- 標準出力への各 Phase の分析結果と最終判定
- `output/verify_fx_fy_swap_<tire_model>.png`

### 主要変数

- `test_Fz_vals = [200, 500, 800, 1100, 1400]`
- `test_IA_val = 0`
- `test_TSTC_val = 30`
- `sa_thresh = 0.5`, `sr_thresh = 0.01`

## 4. 依存関係

- プロジェクト内関数
  - `loadTireData`
  - `extractTargets`
- 学習済みモデル内変数
  - `cr_trained`, `cr_norm`, `db_trained`, `db_norm`
- MATLAB 組み込み / Deep Learning Toolbox
  - `dir`, `fullfile`, `load`, `predict`, `corr`, `mean`, `std`, `max`, `min`, `scatter`, `plot`, `saveas`, `sgtitle`
- ローカル関数
  - `ternary`

## 5. 処理の流れ

### 5.1 ユーザ設定

1. タイヤモデル名と CSV ディレクトリを設定します。
2. ゼロスリップ検証用の荷重、キャンバー、温度条件を定義します。

### 5.2 Phase 1: 生 CSV データの物理的整合性チェック

1. `Cr_<tire_model>_*.csv` と `Db_<tire_model>_*.csv` を検索します。
2. 各ファイル一覧を表示します。
3. `loadTireData` を使って DriveBrake と Cornering の全データを読み込みます。
4. `Fx`, `Fy`, `SR`, `SA` の統計量を表示します。
5. DriveBrake では `corr(SR, Fx)` と `corr(SR, Fy)` を比較し、`Fx` が縦力らしいかを判定します。
6. Cornering では `corr(SA, Fy)` と `corr(SA, Fx)` を比較し、`Fy` が横力らしいかを判定します。

### 5.3 Phase 2: `loadTireData` パイプライン検証

1. もう一度 `loadTireData` の戻り値を `db_loaded`, `cr_loaded` に保持します。
2. 読み込み後テーブルの列名を表示します。
3. DriveBrake の先頭 10 行について `Fx` と `Fy` を表示し、パイプラインの変換が破綻していない前提を確認します。
4. `db_loaded.Fx` と `db_loaded.Fy` の先頭 10 行が過度に一致していないかを数え、入替疑いの簡易チェックをします。

### 5.4 Phase 3: 正規化パラメータ整合性チェック

1. 学習済みモデルが両方存在しなければ Phase 3 以降をスキップします。
2. `db_norm.output_mean/std` と `extractTargets(db_loaded)` の平均・標準偏差を比較します。
3. Cornering 側でも `cr_norm.output_mean/std` と `extractTargets(cr_loaded)` の平均を比較します。

### 5.5 Phase 4: モデル出力の入替シミュレーション

1. `SA=0, SR=0` 条件で DriveBrake モデルを推論し、通常出力と `Fx↔Fy` 入替後出力を並べます。
2. `SR` スイープで通常出力と入替出力を比較し、DriveBrake 期待挙動 `Fxレンジ大・Fyレンジ小` に合うかを見ます。
3. `SA` スイープで Cornering モデルを比較し、Cornering 期待挙動 `Fyレンジ大・Fxレンジ小` に合うかを見ます。

### 5.6 Phase 5: 生データのゼロスリップ区間分析

1. DriveBrake の `|SA| < 0.5` 行で `Fy` と `Fx` の統計を見ます。
2. DriveBrake の `|SR| < 0.01` 行で `Fx` と `Fy` の統計を見ます。
3. Cornering の `|SR| < 0.01` 行数を確認します。
4. Cornering データ全体の `SR` 分布を表示し、`SR≈0` に集中しているかを確認します。

### 5.7 Phase 6: 可視化

1. 2×4 サブプロットで、生データ相関とモデル出力比較を描きます。
2. 学習済みモデル由来の `Y_sr`, `Y_sa` が存在する場合のみ下段を描画します。
3. `output` ディレクトリがなければ作成し、PNG を保存します。

### 5.8 最終判定

1. Phase 1 の結果を要約表示します。
2. 生データ段階で入替が見えない場合は、問題原因候補を「正規化、モデルアーキテクチャ、学習過程」に絞る文章を出します。
3. 逆に異常があれば `loadTireData.m` のカラムマッピング確認を促します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- 生データ、前処理後データ、学習済みモデル出力の 3 層を横断して `Fx/Fy` 入替を切り分けます。
- Phase 3 以降はモデルファイルが存在する場合のみ実行されます。
- コメント上の「Phase 5: 生データのSA/SR相関分析」に加えて、実装では可視化 Phase と最終判定も含まれます。

### `ternary(cond, true_val, false_val)`

- `cond` が真なら `true_val`、偽なら `false_val` を返す単純な分岐ヘルパーです。
- 判定文や一致/不一致の表示文字列生成に使われます。

## 7. 注意点

- 生 CSV 読み込みにも `loadTireData` を使っているため、完全な意味で「生の列マッピング未加工値」を見ているわけではありません。
- Phase 2 の先頭 10 行比較では、`db_all` と `db_loaded` を厳密に行単位比較しているわけではなく、表示確認が中心です。
- 学習済みモデルが無い場合、Phase 3〜6 はスキップされます。
