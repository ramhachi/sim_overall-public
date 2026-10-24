# `matlab/dl_tire_model/validate_zero_slip_behavior.m`

## 1. 概要

`validate_zero_slip_behavior.m` は、学習済み Cornering モデルと DriveBrake モデルについて、ゼロスリップ近傍での出力が物理的に妥当かを検証するスクリプトです。3 種類のテスト、集計表示、判定、可視化、PNG 保存を行います。

## 2. ファイルの役割

このスクリプトの役割は、`SA=0, SR=0` で力・モーメントがゼロ近傍になるか、`SA=0` では主に `Fx`、`SR=0` では主に `Fy` が応答するかを確認することです。コメントでも、分離学習により以前のバイアス問題が解決されたかを検証すると明記されています。

## 3. インターフェース

### スクリプトとしての入力

- 読み込む学習済みモデル
  - `output/cornering_<tire_model>.mat`
  - `output/drivebrake_<tire_model>.mat`
- 既定設定
  - `tire_model = 'Hoosier_18p0X6p0-10_R20'`
  - `test_Fz = [200, 500, 800, 1100, 1400]`
  - `test_IA = [0, 2, -2]`
  - `test_TSTC = 30`

### スクリプトとしての出力

- 標準出力への各テスト結果、統計、PASS/FAIL 判定
- `output/zero_slip_validation_<tire_model>.png`

### 判定閾値

- `threshold_F = 50` [N]
- `threshold_M = 5` [Nm]
  - ただし、この閾値で実際に判定しているのは `Fx` と `Fy` で、`Mz` の PASS/FAIL 判定には使っていません。

## 4. 依存関係

- 学習済みモデル内の変数
  - Cornering: `cr_trained`, `cr_norm`
  - DriveBrake: `db_trained`, `db_norm`
- MATLAB 組み込み / Deep Learning Toolbox
  - `load`, `exist`, `predict`, `mean`, `std`, `max`, `linspace`, `figure`, `subplot`, `plot`, `yline`, `sgtitle`, `saveas`
- ローカル関数
  - `judge_str`

## 5. 処理の流れ

### 5.1 ユーザ設定

1. タイヤモデル名を設定します。
2. 荷重、キャンバー角、表面温度の検証条件を定義します。

### 5.2 モデル読み込み

1. `output/cornering_<tire_model>.mat` と `output/drivebrake_<tire_model>.mat` のパスを作ります。
2. どちらかが存在しなければ `error` で停止します。
3. `load` でモデルを読み込み、ファイル名を表示します。

### 5.3 Test 1: `SA=0`, `SR=0`

1. すべての `Fz` と `IA` の組み合わせについて入力 `X_zero = [0, 0, Fz, IA, test_TSTC]` を作ります。
2. Cornering モデルでは `cr_norm` を使って正規化し、`predict` 後に逆正規化します。
3. DriveBrake モデルでも同様に `db_norm` を使って推論します。
4. `Fx`, `Fy`, `Mz` を `test_results_zero` に蓄積し、表形式で表示します。
5. 平均絶対値・標準偏差・最大絶対値を出力します。
6. `max(abs(...)) < threshold_F` で `Fx`, `Fy` の PASS/FAIL を判定します。

### 5.4 Test 2: `SA=0`, `SR` 変化

1. `SR_range = linspace(-0.2, 0.2, 41)` を作ります。
2. 固定条件 `Fz=800`, `IA=0`, `TSTC=30` で入力行列 `X_test2` を構成します。
3. DriveBrake モデルで一括推論し、`Fx` の範囲と `Fy` のゼロ近傍性を確認します。
4. `max(abs(Y_db2(:,2))) < threshold_F` で `Fy` の判定を行います。

### 5.5 Test 3: `SR=0`, `SA` 変化

1. `SA_range = linspace(-12, 12, 49)` を作ります。
2. 固定条件 `Fz=800`, `IA=0`, `TSTC=30` で入力行列 `X_test3` を構成します。
3. Cornering モデルで一括推論し、`Fy` の範囲と `Fx` のゼロ近傍性を確認します。
4. `max(abs(Y_cr3(:,1))) < threshold_F` で `Fx` の判定を行います。

### 5.6 可視化

1. 2×3 サブプロットで Test 2 と Test 3 の `Fx`, `Fy`, `Mz` を描きます。
2. `Fy` と `Fx` のゼロ近傍判定図には閾値線を描き、PASS/FAIL を subtitle で付けます。
3. `output` ディレクトリがなければ作成し、PNG を保存します。

### 5.7 最終判定

1. `cr_fx_ok && cr_fy_ok && db_fx_ok && db_fy_ok && fy_ok_test2 && fx_ok_test3` を `all_pass` とします。
2. 各テスト結果と時間を表示し、総合 PASS/FAIL を出力します。

## 6. 関数・ローカル関数詳細

### スクリプト本体

- Cornering モデルと DriveBrake モデルを分けて読み込み、それぞれに適したゼロスリップ検証を行います。
- `Mz` は統計表示と描画には使われますが、総合 PASS 条件には含まれていません。

### `judge_str(pass)`

- 入力 `pass` が真なら `'✓ PASS'`、偽なら `'✗ FAIL'` を返します。
- 表示文字列を統一するためだけのローカルヘルパーです。

## 7. 注意点

- 読み込む `.mat` に `cr_norm` / `db_norm` や `cr_trained` / `db_trained` が含まれる前提です。
- 閾値 `threshold_M` は定義されますが、`Mz` の PASS/FAIL 判定には実際には使われません。
- 出力保存先は作成しますが、モデルファイルが無い場合は途中で停止します。
